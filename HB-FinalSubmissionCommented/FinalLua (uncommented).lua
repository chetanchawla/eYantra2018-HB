--* Team Id : HB#5374
--* Author List : Divyansh Malhotra, Chetan Chawla, Himanshu, Yashvi Gulati
--* Filename: 5374_progress_task.lua
--* Theme: Hungry Bird
--* Classes: Edrone

aruco={}   --To save aruco marker orientations of all the aruco markers
ids={}     -- To save the ids of the aruco markers present in the scene

--Function Name: sysCall_init
--Logic: Initialises all handles and variables and sets up publishers and subscribers required
function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDroneBase')
    collection_handles= sim.getCollectionHandle('Obstacles')

    em_drone = sim.getObjectHandle('eDrone_Non_Dynamic')
    initial_handle=sim.getObjectHandle('initial_waypoint')
    --changing handles ---------------------------------------------------------------------
    red_front=sim.getObjectHandle('Front_hoop3')
    red_back=sim.getObjectHandle('Back_hoop3')
    yellow_front=sim.getObjectHandle('Front_hoop2')
    yellow_back=sim.getObjectHandle('Back_hoop2')
    green_front=sim.getObjectHandle('Front_hoop1')
    green_back=sim.getObjectHandle('Back_hoop1')
    -- Assigning obstacles handles
    green=sim.getObjectHandle('Orientation_hoop1')  --sal tree's hoop
    yellow=sim.getObjectHandle('Orientation_hoop2') --mango tree's hoop
    red=sim.getObjectHandle('Orientation_hoop3')    --cashew tree's hoop
    cashew=sim.getObjectHandle('Position_hoop3')    --cashew tree
    mango=sim.getObjectHandle('Position_hoop2')     --mango tree
    sal=sim.getObjectHandle('Position_hoop1')       --sal tree
    nofruit1=sim.getObjectHandle('obstacle_1')      --non-fruit tree1
    nofruit2=sim.getObjectHandle('obstacle_2')      --non-fruit tree2
    nofruit3=sim.getObjectHandle('obstacle_3')      --non-fruit tree3
    --variables
    no_of_obstacles = 6
    -- obstacles_handles = {}
    -- for i=1,no_of_obstacles do
    --     table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    -- end
    emulate=false
    noFruitCounter=0

    -----------Add other required handles here----------------
    -- initial_handle=sim.getObjectHandle('initial_waypoint')
    -- goal1_handle=sim.getObjectHandle('goal_1')
    -- goal2_handle=sim.getObjectHandle('goal_2')
    compute_path_flag=false--the flag is initially set to false
    goals={initial_handle,red_front,yellow_front,red_front,green_front,red_front,initial_handle} -- A lua list of the handles is made to contain all the goals that are to be reached in sequential manner
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------
    t=simOMPL.createTask('t')
    --Taking the x-y state space constraint as the boundaries of Vision Sensor Field View and z axis as the maximum height from all the obstacles
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2,-2,0.0},{2,2,1.6},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})--eDrone_outer boundary is used as the collider and the collection handles having the obstacles group is taken as collidee
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    counter=0
    make_path=simROS.subscribe('/checking','geometry_msgs/PoseArray','initiatepath',1) -- Subscriber is called whenever the python code asks for a new path. The message is os PoseArray type and function initiatepath is called
    aruco_sub=simROS.subscribe('/aruco_marker_publisher/markers','aruco_msgs/MarkerArray','aruco_callback') --to find the orientation of the aruco markers in a scene  
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')            --to find the whycon position values of the whycon marker in the scene
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
    ---------------------------------------------------------------------------------------


    -- scale_factor = {} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    --scale factor added directly in the function packData
    no_of_path_points_required=14-- Add no of path points you want from one point to another

end

--Function Name: initiatepath
--Logic: It is a callback to /checking. It initiate and gives the new path
function initiatepath(msg)
    --initiate and gives the new path
    if(counter<(table.getn(goals))) then --checking if all the goal points are reached
        compute_path_flag=true --turns the compute_path_flag true to get new path
        counter=counter+1 --turns up the counter of paths that were needed
    end
end
--Function Name: visualizpath
--Logic: This function can be used to visualize the path you compute. This function takes path points as the argument...
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],-path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],-path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

--Function Name: getpose
--Logic: It returns the position and orientation of a handle with respect to a reference
function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],-position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end

--Function Name: packdata
--Logic: This function is used to send the Path computed in the real_world to whycon_world after transformation. Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
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
        pose.position.z = 30.54 - (11.44*pose.position.z)
        sender.poses[math.floor(i/7) + 1] = pose


        --------------------------------------------------------------------------------------------------------------------
    end
    if goals[counter+1] == red_front then
            goal_back = red_back
    end
    if goals[counter+1] == yellow_front then
        goal_back = yellow_back
    end
    if goals[counter+1] == green_front then
        goal_back = green_back
    end
    if goals[counter+1] == initial_handle then
        goal_back = initial_handle
    end
    poseb=getpose(goal_back,-1)
    a = {x = poseb[4], y = poseb[5], w = poseb[6], z = poseb[7]}
    b = {x = poseb[1], y = poseb[2], z = poseb[3]}
    pose = {position = b, orientation = a, }
    pose.position.x = -7.557*pose.position.x
    pose.position.y = -7.604*pose.position.y
    pose.position.z = 30.54 - (11.44*pose.position.z)
    sender.poses[no_of_path_points_required + 1] = pose
    return sender
end


--Function Name: compute_and_send_path
--Logic: This function is used to compute and publish the path to path_planninglpy
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

--Function Name: sysCall_actuation
--Logic: It starts the process of path planning when requested
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

--Function Name: aruco_callback
--Logic: It is a callback to /aruco_marker_publisher/markers. It finds the orientation of all hoops
function aruco_callback(msg)
    
    len = #msg.markers --len=no. of aruco markers in the scene
    for i=1,len do
        ids[i]=msg.markers[i].id
        --The signs have been set in accordance to our camera orientation with respect to the flex:(-x,y,-z,w)
        val={-msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,-msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w}
        aruco[i]=val 
        --the orientation of the specfic tree can be found out by utilising the index of its id in 'ids' and accessing that index's quaternion from 'aruco'
    end
end
-- Function Name: whycon_callback
-- Input: msg- whycon position coordinate 
-- Logic: Get the position of the whycon marker. Only one marker is present at once in the scene.
function whycon_callback(msg)
     
    wx=msg.poses[1].position.x
    wy=msg.poses[1].position.y
    wz=msg.poses[1].position.z
    -- Conversion of whycon ccordinates to real world
    wx = wx/(-7.557)  
    wy = wy/(7.604) 
    wz=2.66931-0.08738*wz
    if emulate then
        sim.setObjectPosition(em_drone,-1,{wx,wy,wz})
    end
end
--* Function Name: key_callback
--* Input: msg- integer value corresponding to a key clicked on the keyboard 
--* Logic: Read key input to set or unset position and orientation of food and non-food trees and then do accordingly
function key_callback(msg)
    
    if msg.data == 1 then
        for j =1,#ids do  -- run for all aruco markers scanned on the scene
            if ids[j] == 0 then
                --set orientation of sal tree
                sim.setObjectQuaternion(green,-1,aruco[j])
            elseif ids[j] == 1 then
                --set orientation of mango tree
                sim.setObjectQuaternion(yellow,-1,aruco[j])
            elseif ids[j] == 2 then
                --set orientation of cashew tree
                sim.setObjectQuaternion(red,-1,aruco[j])
            end
        end
    
    elseif msg.data==2 then
        --set position of sal tree
        sim.setObjectPosition(sal,-1,{wx,wy,wz})
    
    elseif msg.data==3 then
        --set position of mango tree
        sim.setObjectPosition(mango,-1,{wx,wy,wz})
    
    elseif msg.data==4 then
        --set position of cashew tree
        sim.setObjectPosition(cashew,-1,{wx,wy,wz})
        
    elseif msg.data==5 then
        --set position of non-fruit tree
        noFruitCounter=noFruitCounter+1
        if noFruitCounter==1 then
            sim.setObjectPosition(nofruit1,-1,{wx,wy,wz+0.3})
        elseif noFruitCounter==2 then
            sim.setObjectPosition(nofruit2,-1,{wx,wy,wz+0.3})
        elseif noFruitCounter==3 then
            sim.setObjectPosition(nofruit3,-1,{wx,wy,wz+0.3})
        end
    elseif msg.data==6 then
        -- set all trees outside the arena before setting their positions
        --setting wx,wy,wz to values that are outside of arena
        wx=2.9
        wy=0.5080
        wz=0.6779
        sim.setObjectPosition(sal,-1,{wx,wy,wz})        --set sal outside the arena
        sim.setObjectPosition(mango,-1,{wx+1,wy,wz})    --set mango outside the arena
        sim.setObjectPosition(cashew,-1,{wx+2,wy,wz})   --set cashew outside the arena
        sim.setObjectPosition(nofruit1,-1,{wx+3,wy,wz})  --set non-fruit 1 tree outside the arena
        sim.setObjectPosition(nofruit2,-1,{wx+3,wy,wz})  --set non-fruit 1 tree outside the arena
        sim.setObjectPosition(nofruit3,-1,{wx+3,wy,wz})  --set non-fruit 1 tree outside the arena
    elseif msg.data==8 then
        sim.setObjectPosition(initial_handle,-1,{wx+0.3,wy+0.5,wz+0.8})
        emulate=true
        sim.setObjectOrientation(initial_handle,-1,{0,0,0})
        sim.setObjectOrientation(red_front,-1,{0,0,0})
        sim.setObjectOrientation(red_back,-1,{0,0,0})
        sim.setObjectOrientation(yellow_front,-1,{0,0,0})
        sim.setObjectOrientation(yellow_back,-1,{0,0,0})
        sim.setObjectOrientation(green_front,-1,{0,0,0})
        sim.setObjectOrientation(green_back,-1,{0,0,0})
        sim.setObjectOrientation(nofruit1,-1,{0,0,0})
        sim.setObjectOrientation(nofruit2,-1,{0,0,0})
        sim.setObjectOrientation(nofruit3,-1,{0,0,0})
    end
end