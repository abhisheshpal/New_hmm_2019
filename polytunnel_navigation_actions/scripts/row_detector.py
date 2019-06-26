#!/usr/bin/env python
from __future__ import division
import rospy, sys, yaml, rospkg
import numpy as np
import std_srvs

from std_srvs.srv import SetBool
from sklearn.externals import joblib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from polytunnel_navigation_actions.msg import Obstacle, ObstacleArray
from sklearn.cluster import DBSCAN
from sklearn import linear_model


class row_detector(object):
    
    
    def __init__(self, config_ellipses, ep):
        
        self.config = config_ellipses
        self.ep = ep
        self.is_active= False
        
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        self.path_err_pub = \
        rospy.Publisher("/row_detector/path_error", Pose2D, queue_size=10)
        
        self.poles_pub = \
        rospy.Publisher("/row_detector/poles", ObstacleArray, queue_size=10)
        
        self.obstacles_pub = \
        rospy.Publisher("/row_detector/obstacles", ObstacleArray, queue_size=10)

        rospy.Service('/row_detector/activate_detection', SetBool, self.activate_callback)
        
        
    def activate_callback(self, req):

        self.is_active=req.data
        ans = std_srvs.srv.SetBoolResponse()
        ans.success = True
        ans.message = 'Detection activated' if req.data else 'Detection de-activated'

        return ans
        
        
    def scan_callback(self, scan):

        if self.is_active:
            
            angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
            ranges = np.array(scan.ranges)
            
            xs = ranges * np.cos(angles)
            ys = ranges * np.sin(angles)
            
            success = True        
            lines_x = []
            lines_y = []
            pole_array = []
            obstacle_array = []
            
            for ellipse in self.config:
                
                data = self.ep.process(ellipse, xs, ys)
                
                if ellipse["detect_rows"] and ellipse["fit_required"] and not data["fitted"]:
                    success = False
                
                if ellipse["detect_rows"] and data["fitted"]:
                    lines_x.append(data["lines_x"])
                    lines_y.append(data["lines_y"])
                    
                if ellipse["publish_poles"]:
                    pole_array.append(data["pole_array"])
                    
                if ellipse["publish_obstacles"]:
                    obstacle_array.append(data["obstacle_array"])
    
            lines_x = self.flatten(lines_x)        
            lines_y = self.flatten(lines_y) 
            self.pole_array = self.flatten(pole_array)        
            self.obstacle_array = self.flatten(obstacle_array)      
    
            if success:            
                self.error_y, self.error_theta = self.calc_error(lines_x, lines_y)    
            else:
                self.error_y = np.nan; self.error_theta = np.nan
                
            self.publish_msgs()
            
            
    def flatten(self, list_):
        return [item for sublist in list_ for item in sublist]
    
    
    def calc_error(self, lines_x, lines_y):
        
        try:
            centre_x = np.mean(np.concatenate(lines_x, axis=1), axis=1)
            centre_y = np.mean(np.concatenate(lines_y, axis=1), axis=1)
    
            error_y = centre_y[np.argmin(np.sqrt(centre_x**2 + centre_y**2))]
            error_theta = np.arctan2((centre_y[-1] - centre_y[0]), (centre_x[-1] - centre_x[0])) 
            
            if error_theta > 0.79:
                error_y = np.nan; error_theta = np.nan  
                
        except:
            error_y = np.nan; error_theta = np.nan  
            
        return error_y, error_theta    
        
        
    def publish_msgs(self):
        
        path_error = Pose2D()
        path_error.x = np.nan
        path_error.y = self.error_y
        path_error.theta = self.error_theta
        
        poles = ObstacleArray()
        poles.obstacles = self.pole_array
        
        obstacles = ObstacleArray()
        obstacles.obstacles = self.obstacle_array
        
        self.path_err_pub.publish(path_error)
        self.poles_pub.publish(poles)
        self.obstacles_pub.publish(obstacles)
#####################################################################################


#####################################################################################
class ellipse_processor(object):
    
    
    def __init__(self, config_common, clf):
        
        self.config = config_common
        self.clf = clf
        
        self.db = DBSCAN(eps=self.config["eps"], 
                         min_samples=self.config["min_samples_dbscan"])
        
        self.ransac = linear_model.RANSACRegressor(
                      residual_threshold=self.config["residual_threshold"], 
                      min_samples=self.config["min_samples_ransac"])
                      
                      
    def process(self, ellipse, xs, ys):
        
        self.poles_identified = False
        self.data = {"fitted": False, "lines_x": [], "lines_y": []}
        
        indices_ellipse = np.where(((xs - ellipse["x"])**2 / ellipse["rx"]**2 + (ys - ellipse["y"])**2 / ellipse["ry"]**2) <= 1)[0]
        self.xs = xs[indices_ellipse]
        self.ys = ys[indices_ellipse]
        
        self.xs_all = self.xs
        self.ys_all = self.ys
        
        self.classify()
        self.data["pole_array"] = self.pole_array
        self.data["obstacle_array"] = self.obstacle_array
        
        if self.poles_identified:
            if ellipse["split"]:
                self.fit_two_lines(ellipse)
            else:
                self.fit_one_line(ellipse)
        
        return self.data
        
        
    def classify(self):
        
        pole_clusters = []
        muys = []          
        self.pole_array = []
        self.obstacle_array = []
        
        X = np.vstack((self.xs, self.ys)).T
        
        try:
            self.db.fit(X)
        except:
            return
            
        labels = self.db.labels_        
        
        for label in set(labels):
            if label == -1:
                continue
            
            class_member_mask = (labels == label)
            cluster = X[class_member_mask]
            
            cluster_xs = cluster[:, 0]
            cluster_ys = cluster[:, 1]
            
            self.mux = np.mean(cluster_xs)
            self.muy = np.mean(cluster_ys)
            
            dxs = cluster_xs - self.mux
            dys = cluster_ys - self.muy
            self.eds = np.sqrt(dxs**2 + dys**2)
            
            obstacle = Obstacle()
            obstacle = self.fill_obstacle_msg(obstacle)
            
            features, bins = np.histogram(self.eds, bins=90, range=(0, 1.45))
            is_pole = self.clf.predict(features.reshape(1, -1))
            
            if is_pole:
                pole_clusters.append(cluster)
                muys.append(self.muy)
                self.pole_array.append(obstacle)
            else:
                self.obstacle_array.append(obstacle)
        
        try:
            pole_clusters = np.concatenate(pole_clusters, axis=0)
            self.xs = pole_clusters[:, 0]
            self.ys = pole_clusters[:, 1]
            self.muys = np.array(muys)            
            
            self.poles_identified = True
            
        except:
            pass
        
        
    def fill_obstacle_msg(self, msg):
        
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.pose.x = self.mux
        msg.pose.y = self.muy
        msg.pose.theta = np.nan
        msg.radius = np.max(self.eds)
        
        return msg
    
    
    def fit_one_line(self, ellipse):
        
        if len(self.muys) < ellipse["min_num_poles"]:
            line_fitted = self.fit_line(self.xs_all, self.ys_all)
        else:
            line_fitted = self.fit_line(self.xs, self.ys)
            
        if line_fitted:
            self.data["lines_x"].append(self.line_x)
            self.data["lines_y"].append(self.line_y)
            self.data["fitted"] = True

        
    def fit_two_lines(self, ellipse):
        
        num_poles_region_1 = len(np.where(self.muys >= ellipse["y"])[0])
        num_poles_region_2 = len(np.where(self.muys < ellipse["y"])[0])
        
        if num_poles_region_1 < ellipse["min_num_poles"]: 
            indices_region_1 = np.where(self.ys_all >= 0)[0]
            xs_region_1 = self.xs_all[indices_region_1]
            ys_region_1 = self.ys_all[indices_region_1]  
        else:
            indices_region_1 = np.where(self.ys >= 0)[0]
            xs_region_1 = self.xs[indices_region_1]
            ys_region_1 = self.ys[indices_region_1]  
            
        if num_poles_region_2 < ellipse["min_num_poles"]: 
            indices_region_2 = np.where(self.ys_all < 0)[0]
            xs_region_2 = self.xs_all[indices_region_2]
            ys_region_2 = self.ys_all[indices_region_2]
        else:
            indices_region_2 = np.where(self.ys < 0)[0]
            xs_region_2 = self.xs[indices_region_2]
            ys_region_2 = self.ys[indices_region_2]
        
        line_fitted_region_1 = self.fit_line(xs_region_1, ys_region_1)
        if line_fitted_region_1:
            self.data["lines_x"].append(self.line_x)
            self.data["lines_y"].append(self.line_y)
        
        line_fitted_region_2 = self.fit_line(xs_region_2, ys_region_2)
        if line_fitted_region_2:
            self.data["lines_x"].append(self.line_x)
            self.data["lines_y"].append(self.line_y)
            
        if line_fitted_region_1 and line_fitted_region_2:
            self.data["fitted"] = True
            

    def fit_line(self, xs, ys):
        
        line_fitted = False
        X = xs.reshape(-1, 1)
        for try_ in range(self.config["num_attempted_fits"]):
            
            try:
                self.ransac.fit(X, ys)
                x_inlier = xs[self.ransac.inlier_mask_]
                
                self.line_x = np.linspace(np.min(x_inlier), np.max(x_inlier), self.config["line_granularity"])[:, np.newaxis]
                self.line_y = self.ransac.predict(self.line_x).reshape(-1, 1)
                     
                line_fitted = True
                
            except:
                pass
            
            if line_fitted:
                break
    
        return line_fitted
#####################################################################################


#####################################################################################
if __name__ == "__main__":
    
    rospy.init_node("row_detector", anonymous=True)
    
    if len(sys.argv) < 3:
        rospy.logerr("usage is row_detector.py path_to_common_cfg_yaml path_to_ellipse_cfg_yaml")
        exit()
    else:
        print sys.argv
        common_cfg_path = sys.argv[1]
        ellipse_cfg_path = sys.argv[2]
        
    with open(common_cfg_path, 'r') as f:
        config_common = yaml.load(f)
        
    with open(ellipse_cfg_path, 'r') as f:
        config_ellipses = yaml.load(f)
        
    package_dir = rospkg.RosPack().get_path("polytunnel_navigation_actions")   
    clf = joblib.load(package_dir + "/resources/svm/svm.pkl" )

    ep = ellipse_processor(config_common, clf)    
    rd = row_detector(config_ellipses, ep)
    
    rospy.spin()
#####################################################################################