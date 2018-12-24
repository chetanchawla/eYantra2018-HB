--* Team Id : #5374
--* Author List : Divyansh Malhotra, Chetan Chawla, Himanshu, Yashvi Gulati
--* Filename: Task2.lua
--* Theme: Hungry Bird
--* Functions: sysCall_init,aruco_callback,whycon_callback,key_callback,sysCall_actuation,sysCall_sensing,sysCall_cleanup
--* Global Variables: aruco,ids


-- This script is used for realtime emulation of the environment in V-REP
aruco={}   --To save aruco marker orientations of all the aruco markers
ids={}     -- To save the ids of the aruco markers present in the scene

--* Function Name: sysCall_init
--* Logic: to get object handles and subscribe to required topics
function sysCall_init()

    -- Object handles 

    green=sim.getObjectHandle('Orientation_hoop1')  --sal tree's hoop
    yellow=sim.getObjectHandle('Orientation_hoop2') --mango tree's hoop
    red=sim.getObjectHandle('Orientation_hoop3')    --cashew tree's hoop
    cashew=sim.getObjectHandle('Position_hoop3')	--cashew tree
    mango=sim.getObjectHandle('Position_hoop2')		--mango tree
    sal=sim.getObjectHandle('Position_hoop1')		--sal tree
    nofruit=sim.getObjectHandle('obstacle_1')		--non-fruit tree

    -- Subscribing to the required topics 
    aruco_sub=simROS.subscribe('/aruco_marker_publisher/markers','aruco_msgs/MarkerArray','aruco_callback') --to find the orientation of the aruco markers in a scene  
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')			--to find the whycon position values of the whycon marker in the scene
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')							--to get input on pressing a key on keyboard in the terminal
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


--* Function Name: aruco_callback
--* Input: msg- aruco orientation quaternion 
--* Logic: Get the orientation(quaternion) of the ArUco marker and save it in 'aruco' and their respective ids in 'ids'
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

--* Function Name: whycon_callback
--* Input: msg- whycon position coordinate 
--* Logic: Get the position of the whycon marker. Only one marker is present at once in the scene.
function whycon_callback(msg)
     
    wx=msg.poses[1].position.x
    wy=msg.poses[1].position.y
    wz=msg.poses[1].position.z
    -- Conversion of whycon ccordinates to real world
    wx = wx/(-7.557)  
    wy = wy/(7.604) 
    wz=2.66931-0.08738*wz
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
        sim.setObjectPosition(nofruit,-1,{wx,wy,wz+0.3})

    elseif msg.data==6 then
    	-- set all trees outside the arena before setting their positions
    	--setting wx,wy,wz to values that are outside of arena
        wx=2.9
        wy=0.5080
        wz=0.6779
        sim.setObjectPosition(sal,-1,{wx,wy,wz})  		--set sal outside the arena
        sim.setObjectPosition(mango,-1,{wx+1,wy,wz})	--set mango outside the arena
        sim.setObjectPosition(cashew,-1,{wx+2,wy,wz})	--set cashew outside the arena
        sim.setObjectPosition(nofruit,-1,{wx+3,wy,wz})	--set non-fruit tree outside the arena
    end
end
