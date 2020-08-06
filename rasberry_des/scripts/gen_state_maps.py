#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import sys
import numpy
# import rospkg
import os


# def get_fwd_map(n_rows, n_nodes, verbose=False):
#     """
#     """
#     # forward state_map
#     state_map_fwd = numpy.eye(sum(n_nodes), k=1)
#     state_map_fwd += numpy.eye(sum(n_nodes), k=-1) * 0.1
#     state_map_fwd[-1,0] += 1
#     state_map_fwd[0,-1] += 0.1
#     if verbose:
#         print (state_map_fwd)
#     return state_map_fwd


# def get_bwd_map(n_rows, n_nodes, verbose=False):
#     """
#     """
#     # backward state_map
#     state_map_bwd = numpy.eye(sum(n_nodes), k=-1)
#     state_map_bwd += numpy.eye(sum(n_nodes), k=1) * 0.1

#     if n_rows < 2:
#         raise Exception("Number of rows must be at least 2")
#     elif n_rows == 2:
#         r = - n_nodes[0]
#         c = n_nodes[0] - 1
#         for i in range(n_rows-1):
#             # set
#             r += n_nodes[i]
#             c += n_nodes[i+1]
#             state_map_bwd[r, c] = 1
#             state_map_bwd[c, r] = 0.1

#     elif n_rows > 2:
#         r = - n_nodes[0]
#         c = n_nodes[0] - 1
#         for i in range(n_rows-1):
#             # set
#             r += n_nodes[i]
#             c += n_nodes[i+1]
#             state_map_bwd[r, c] = 1
#             state_map_bwd[c, r] = 0.1
#             state_map_bwd[r, r - 1] = 0
#             state_map_bwd[r - 1, r] = 0

#         r += n_nodes[n_rows-1]
#         c = n_nodes[0] - 1
#         state_map_bwd[r, c] = 1
#         state_map_bwd[c, r] = 0.1
#         state_map_bwd[r, r - 1] = 0
#         state_map_bwd[r - 1, r] = 0

#     if verbose:
#         print (state_map_bwd)
#     return state_map_bwd


# if __name__ == "__main__":
#     """
#     """
#     if len(sys.argv) < 4:
#         print """Usage: gen_state_maps 'map_name' n_rows n_nodes save_maps
#         e.g. for a topo map with name single_bed with two navigation rows, each with 96 nodes and to save them
#         gen_state_maps 'single_bed' '2' '[96, 96]' 'true'
#         """
#         exit()
#     elif len(sys.argv) == 4:
#         map_name = sys.argv[1] # map_name
#         n_rows = int(sys.argv[2]) # number of rows
#         n_nodes_str = sys.argv[3].strip("[").strip("]").replace(", ", ",").split(",") # number of nodes in each row
#         n_nodes = [int(item) for item in n_nodes_str]
#         save_state_maps = False
#     else:
#         map_name = sys.argv[1] # map_name
#         n_rows = int(sys.argv[2]) # number of rows
#         n_nodes_str = sys.argv[3].strip("[").strip("]").replace(", ", ",").split(",") # number of nodes in each row
#         n_nodes = [int(item) for item in n_nodes_str]
#         save_state_maps = sys.argv[4].lower() == "true"

# #    save_state_maps = False
# #    n_rows = 2
# #    n_nodes = [5, 5]
# #    n_rows = 3
# #    n_nodes = [5, 5, 5]
# #    map_name = "test_bed"

#     fwd_map = get_fwd_map(n_rows, n_nodes)
#     bwd_map = get_bwd_map(n_rows, n_nodes)

#     if save_state_maps:
#         rospack = rospkg.RosPack()
#         dir_path = os.path.join(rospack.get_path("rasberry_des"), "resources", "hmm_topo_state_maps")
#         numpy.savez(os.path.join(dir_path, "%s_fwd_state_map.npz" %(map_name)), state_map=fwd_map)
#         numpy.savez(os.path.join(dir_path, "%s_bwd_state_map.npz" %(map_name)), state_map=bwd_map)


# Abhishesh changes --- To work on just single row operation at a time for forward and backward picking to decrease computational complexity  

def get_fwd_map(n_rows, n_nodes, verbose=False):
    """
    """
    # forward state_map
    state_map_fwd = numpy.eye(sum(n_nodes), k=1)
    state_map_fwd += numpy.eye(sum(n_nodes), k=-1) * 0.1
    state_map_fwd[-1,0] += 1
    state_map_fwd[0,-1] += 0.1
    
    if n_rows < 2:
        raise Exception("Number of rows must be at least 2")
    elif n_rows == 2:
        r = - n_nodes[0]
        c = n_nodes[0] - 1
        for i in range(n_rows-1):
        # set
            r += n_nodes[i]
            c += n_nodes[i+1]
            state_map_fwd[r, c] = 1
            state_map_fwd[c, r] = 0.1
    
    elif n_rows > 2:
        r = - n_nodes[0]
        c = n_nodes[0] - 1
        for i in range(n_rows-1):
            # set
            r += n_nodes[i]
            c += n_nodes[i+1]
            state_map_fwd[r, c] = 1
            state_map_fwd[c, r] = 0.1
            state_map_fwd[r, r - 1] = 0
            state_map_fwd[r - 1, r] = 0
        
        r += n_nodes[n_rows-1]
        c = n_nodes[0] - 1
        state_map_fwd[r, c] = 1
        state_map_fwd[c, r] = 0.1
        state_map_fwd[r, r - 1] = 0
        state_map_fwd[r - 1, r] = 0
    
    if verbose:
        print (state_map_fwd)
    return state_map_fwd

def get_bwd_map(n_rows, n_nodes, verbose=False):
    """
    """
    # backward state_map
    state_map_bwd = numpy.eye(sum(n_nodes), k=-1)
    state_map_bwd += numpy.eye(sum(n_nodes), k=1) * 0.1

    if n_rows < 2:
        raise Exception("Number of rows must be at least 2")
    elif n_rows == 2:
        r = - n_nodes[0]
        c = n_nodes[0] - 1
        for i in range(n_rows-1):
            # set
            r += n_nodes[i]
            c += n_nodes[i+1]
            state_map_bwd[r, c] = 1
            state_map_bwd[c, r] = 0.1

    elif n_rows > 2:
        r = - n_nodes[0]
        c = n_nodes[0] - 1
        for i in range(n_rows-1):
            # set
            r += n_nodes[i]
            c += n_nodes[i+1]
            state_map_bwd[r, c] = 1
            state_map_bwd[c, r] = 0.1
            state_map_bwd[r, r - 1] = 0
            state_map_bwd[r - 1, r] = 0

        r += n_nodes[n_rows-1]
        c = n_nodes[0] - 1
        state_map_bwd[r, c] = 1
        state_map_bwd[c, r] = 0.1
        state_map_bwd[r, r - 1] = 0
        state_map_bwd[r - 1, r] = 0

    if verbose:
        print (state_map_bwd)
    return state_map_bwd




if __name__ == "__main__":
    """
    """
    if len(sys.argv) < 4:
        print ("""Usage: gen_state_maps 'map_name' n_rows n_nodes save_maps
        e.g. for a topo map with name single_bed with two navigation rows, each with 96 nodes and to save them
        gen_state_maps 'single_bed' '2' '[96, 96]' 'true'
        """)
        # exit()
    elif len(sys.argv) == 4:
        map_name = sys.argv[1] # map_name
        n_rows = int(sys.argv[2]) # number of rows
        n_nodes_str = sys.argv[3].strip("[").strip("]").replace(", ", ",").split(",") # number of nodes in each row
        n_nodes = [int(item) for item in n_nodes_str]
        save_state_maps = False
    else:
        map_name = sys.argv[1] # map_name
        n_rows = int(sys.argv[2]) # number of rows
        n_nodes_str = sys.argv[3].strip("[").strip("]").replace(", ", ",").split(",") # number of nodes in each row
        n_nodes = [int(item) for item in n_nodes_str]
        save_state_maps = sys.argv[4].lower() == "true"

    save_state_maps = True #False
    n_rows = 2
    n_nodes = [5, 5]
    n_rows = 3
    n_nodes = [5, 5, 5]
    map_name = "test_bed"

    fwd_map = get_fwd_map(n_rows, n_nodes)
    bwd_map = get_bwd_map(n_rows, n_nodes)

    # if save_state_maps:
    #     rospack = rospkg.RosPack()
    #     dir_path = os.path.join(rospack.get_path("rasberry_des"), "resources", "hmm_topo_state_maps")
    #     numpy.savez(os.path.join(dir_path, "%s_fwd_state_map.npz" %(map_name)), state_map=fwd_map)
    #     numpy.savez(os.path.join(dir_path, "%s_bwd_state_map.npz" %(map_name)), state_map=bwd_map)

## ---------------------------