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
    
    
    def __init__(self, config, clf):
        
        self.ONE_LINE = config["one_line"]
        self.RX = config["rx"]
        self.RY = config["ry"]
        self.EPS = config["eps"]
        self.MIN_SAMPLES_DBSCAN = config["min_samples_dbscan"]
        self.MIN_SAMPLES_RANSAC = config["min_samples_ransac"]
        self.RESIDUAL_THRESHOLD = config["residual_threshold"]
        self.NUM_ATTEMPTED_FITS = config["num_attempted_fits"]
        self.LINE_GRANULARITY = config["line_granularity"]
        self.MIN_NUM_POLES = config["min_num_poles"]
        self.clf = clf
        self.is_active= False
        
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        
        self.db = DBSCAN(eps=self.EPS, min_samples=self.MIN_SAMPLES_DBSCAN)
        
        self.ransac = linear_model.RANSACRegressor(
        residual_threshold=self.RESIDUAL_THRESHOLD, min_samples=self.MIN_SAMPLES_RANSAC)
        
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
            self.poles_identified = False
            
            angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
            ranges = np.array(scan.ranges)
            
            self.xs = ranges * np.cos(angles)
            self.ys = ranges * np.sin(angles)
            
            self.filter_by_elipse()
            self.xs_all = self.xs
            self.ys_all = self.ys
            self.filter_by_svm()
            
            if self.poles_identified:
                if self.ONE_LINE:
                    error_y, error_theta = self.fit_one_line()
                else:
                    error_y, error_theta = self.fit_two_lines()
            else:
                error_y = np.NaN; error_theta = np.NaN
            
            path_error = Pose2D()
            path_error.x = np.NaN
            path_error.y = error_y
            path_error.theta = error_theta
            
            poles = ObstacleArray()
            poles.obstacles = self.pole_array
            
            obstacles = ObstacleArray()
            obstacles.obstacles = self.obstacle_array
            
            self.path_err_pub.publish(path_error)
            self.poles_pub.publish(poles)
            self.obstacles_pub.publish(obstacles)
            
        
    def filter_by_elipse(self):
        
        indices_elipse = np.where((self.xs**2/self.RX**2 + self.ys**2/self.RY**2) <= 1)[0]
        self.xs = self.xs[indices_elipse]
        self.ys = self.ys[indices_elipse]

        
    def filter_by_svm(self):
        
        X = np.vstack((self.xs, self.ys)).T
        
        try:
            self.db.fit(X)
        except:
            return
            
        labels = self.db.labels_
        
        pole_clusters = []
        pole_xs = []
        pole_ys = []
        self.pole_array = []
        self.obstacle_array = []

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
                pole_xs.append(self.mux)
                pole_ys.append(self.muy)
                self.pole_array.append(obstacle)
            else:
                self.obstacle_array.append(obstacle)
        
        try:
            pole_clusters = np.concatenate(pole_clusters, axis=0)
            self.xs = pole_clusters[:, 0]
            self.ys = pole_clusters[:, 1]
            
            self.pole_xs = np.array(pole_xs)
            self.pole_ys = np.array(pole_ys)
            
            self.poles_identified = True
            
        except:
            pass
        
        
    def fill_obstacle_msg(self, msg):
        
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.pose.x = self.mux
        msg.pose.y = self.muy
        msg.pose.theta = np.NaN
        msg.radius = np.max(self.eds)
        
        return msg
                
        
    def fit_one_line(self):

        if len(self.pole_array) < self.MIN_NUM_POLES: 
            line_fitted = self.fit_line(self.xs_all, self.ys_all)
            self.obstacle_array = []
        else:
            line_fitted = self.fit_line(self.xs, self.ys)
        
        if line_fitted:
            error_y, error_theta = self.calc_error(self.line_x, self.line_y)
        else:
            error_y = np.NaN; error_theta = np.NaN
            
        return error_y, error_theta
        
        
    def fit_two_lines(self):
        
        num_poles_region_1 = len(np.where(self.pole_ys >= 0)[0])
        num_poles_region_2 = len(np.where(self.pole_ys < 0)[0])
        
        if num_poles_region_1 < self.MIN_NUM_POLES: 
            indices_region_1 = np.where(self.ys_all >= 0)[0]
            xs_region_1 = self.xs_all[indices_region_1]
            ys_region_1 = self.ys_all[indices_region_1]  
            self.obstacle_array = []
        else:
            indices_region_1 = np.where(self.ys >= 0)[0]
            xs_region_1 = self.xs[indices_region_1]
            ys_region_1 = self.ys[indices_region_1]  
            
        if num_poles_region_2 < self.MIN_NUM_POLES: 
            indices_region_2 = np.where(self.ys_all < 0)[0]
            xs_region_2 = self.xs_all[indices_region_2]
            ys_region_2 = self.ys_all[indices_region_2]
            self.obstacle_array = []
        else:
            indices_region_2 = np.where(self.ys < 0)[0]
            xs_region_2 = self.xs[indices_region_2]
            ys_region_2 = self.ys[indices_region_2]
            
        line_fitted_region_1 = self.fit_line(xs_region_1, ys_region_1)
        if line_fitted_region_1:
            line_x_region_1 = self.line_x
            line_y_region_1 = self.line_y
        
        line_fitted_region_2 = self.fit_line(xs_region_2, ys_region_2)
        if line_fitted_region_2:
            line_x_region_2 = self.line_x
            line_y_region_2 = self.line_y
            
        if line_fitted_region_1 and line_fitted_region_2:
            centre_x = np.mean(np.hstack((line_x_region_1, line_x_region_2)), axis=1)
            centre_y = np.mean(np.hstack((line_y_region_1, line_y_region_2)), axis=1)
            error_y, error_theta = self.calc_error(centre_x, centre_y)
        else:
            error_y = np.NaN; error_theta = np.NaN
            
        return error_y, error_theta
        
        
    def fit_line(self, xs, ys):
        
        line_fitted = False
        for try_ in range(self.NUM_ATTEMPTED_FITS):
            
            try:    
                self.ransac.fit(xs.reshape(-1, 1), ys)
                x_inlier = xs[self.ransac.inlier_mask_]
                
                self.line_x = np.linspace(np.min(x_inlier), np.max(x_inlier), self.LINE_GRANULARITY)[:, np.newaxis]
                self.line_y = self.ransac.predict(self.line_x).reshape(-1, 1)
                     
                line_fitted = True
                
            except:
                pass
            
            if line_fitted:
                break
            
        return line_fitted
        
        
    def calc_error(self, centre_x, centre_y):
        
        error_y = centre_y[np.argmin(np.sqrt(centre_x**2 + centre_y**2))]
        error_theta = np.arctan2((centre_y[-1] - centre_y[0]), (centre_x[-1] - centre_x[0])) # angle of centre line wrt x-axis
        
        return error_y, error_theta
#####################################################################################


#####################################################################################
if __name__ == "__main__":
    
    rospy.init_node("row_detector", anonymous=True)
    
    if len(sys.argv) < 2:
        rospy.logerr("usage is row_detector.py path_to_config_yaml")
        exit()
    else:
        print sys.argv
        config_path = sys.argv[1]

    with open(config_path, 'r') as f:
        config = yaml.load(f)
        
    package_dir = rospkg.RosPack().get_path("polytunnel_navigation_actions")   
    clf = joblib.load(package_dir + "/resources/svm/svm.pkl" )
    
    rd = row_detector(config, clf)
    
    rospy.spin()
#####################################################################################
