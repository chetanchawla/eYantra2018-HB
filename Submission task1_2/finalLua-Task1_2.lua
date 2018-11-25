function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDroneBase')
    collection_handles= sim.getCollectionHandle('Obstacles')

    -- Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------
    initial_handle=sim.getObjectHandle('initial_waypoint')
    goal1_handle=sim.getObjectHandle('goal_1')
    goal2_handle=sim.getObjectHandle('goal_2')
    compute_path_flag=false--the flag is initially set to false
    goals={initial_handle,goal1_handle,goal2_handle,initial_handle} -- A lua list of the handles is made to contain all the goals that are to be reached in sequential manner
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------
    t=simOMPL.createTask('t')
    --Taking the x-y state space constraint as the boundaries of Vision Sensor Field View and z axis as the maximum height from all the obstacles
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-3.2,-2.5,0.0},{3.2,2.5,1.6},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})--eDrone_outer boundary is used as the collider and the collection handles having the obstacles group is taken as collidee
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints

    counter=0
    make_path=simROS.subscribe('/checking','geometry_msgs/PoseArray','initiatepath',1) -- Subscriber is called whenever the python code asks for a new path. The message is os PoseArray type and function initiatepath is called

    ---------------------------------------------------------------------------------------


    -- scale_factor = {} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    --scale factor added directly in the function packData
    no_of_path_points_required=25-- Add no of path points you want from one point to another

end

function initiatepath(msg)
    --initiate and gives the new path
    if(counter<(table.getn(goals))) then --checking if all the goal points are reached
        compute_path_flag=true --turns the compute_path_flag true to get new path
        counter=counter+1 --turns up the counter of paths that were needed
    end
end
-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end

-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = path[i+3], y = path[i+4], w = path[i+5], z = path[i+6]}
        b = {x = path[i], y = path[i+1], z = path[i+2]}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------
        RealToWhycon={-7.557,-7.604,18.97} -- scale factors
        setZ=55.60 --the ground level coordinate in z axis of whycon is 55.60 for 0 z axis coordinate in real world coordinates
        pose.position.x = -7.557*pose.position.x
        pose.position.y = -7.604*pose.position.y
        pose.position.z = 55.60 - (18.97*pose.position.z)
        sender.poses[math.floor(i/7) + 1] = pose


        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    return sender
end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path
    r,path=simOMPL.compute(t,10,-1,no_of_path_points_required)

    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
    end
    return r
end


function sysCall_actuation()
    
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
   if compute_path_flag == true then
        -- Getting startpose
        start_pose = getpose(goals[counter],-1) --Initial point is taken from one goal (current setpoint) to other as taking initial point would increase the error propagatively
        -- Getting the goalpose
        goal_pose = getpose(goals[counter+1],-1) --Final point is the next goal
        -- Setting start state
        simOMPL.setStartState(t,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t)
        if(status == true) then -- path computed
            compute_path_flag = false
        end
    end 
    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end


function sysCall_sensing()

end



function sysCall_cleanup()

end

