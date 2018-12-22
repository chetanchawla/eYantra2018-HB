-- This script is used for realtime emulation of the environment in V-REP
wx=2.9
wy=0.5080
wz=0.6779
ax=0.0
ay=0.0
az=0.0
aw=0.0
aruco={}
ids={}
function sysCall_init()

    -- Add required handles here

    green=setObjectHandler('Orientation_hoop3')
    yellow=setObjectHandler('Orientation_hoop2')
    red=setObjectHandler('Orientation_hoop1')
    cashew=setObjectHandler('Position_hoop3')
    mango=setObjectHandler('Position_hoop2')
    sal=setObjectHandler('Position_hoop1')
    nofruit=setObjectHandler('obstacle_1')

    -- Subscribing to the required topics 
    --aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    aruco_sub=simROS.subscribe('/aruco_marker_publisher/markers','aruco_msgs/MarkerArray','aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion
    len = #msg.poses
    for i=1,len do
        ids[i]=msg.markers[i].id
        val={msg.markers[i].pos.pos.orientation.x,msg.markers[i].pos.pos.orientation.y,msg.markers[i].pos.pos.orientation.z,msg.markers[i].pos.pos.orientation.w}
        aruco[i]=val
    end
    print(ids)
    print(aruco)

end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    wx=msg.poses[0].position.x
    wy=msg.poses[0].position.y
    wz=msg.poses[0].position.z
    wx = -7.557*wx
    wy = -7.604*wy
    wz = 55.60 - (18.97*wz)
end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    if msg == 1 then
        for j =1,#ids do
            if ids[j] == 0 then
                sim.setObjectQuaternion(green,sal,aruco[j])
            elseif ids[j] == 1 then
                sim.setObjectQuaternion(yellow,mango,aruco[j])
            elseif ids[j] == 2 then
                sim.setObjectQuaternion(red,cashew,aruco[j])
            end
        end
    elseif msg==2 then
        sim.setObjectPosition(sal,-1,{wx,wy,wz})
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg==3 then
        sim.setObjectPosition(mango,-1,{wx,wy,wz})
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg==4 then
        sim.setObjectPosition(cashew,-1,{wx,wy,wz})
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg==5 then
        sim.setObjectPosition(nofruit,-1,{wx,wy,wz})
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg==6 then
    	wx=2.9
    	wy=0.5080
        wz=0.6779
    	sim.setObjectPosition(sal,-1,{wx,wy,wz})
    	sim.setObjectPosition(mango,-1,{wx+1,wy,wz})
    	sim.setObjectPosition(cashew,-1,{wx+2,wy,wz})
    	sim.setObjectPosition(nofruit,-1,{wx+3,wy,wz})
    end
end