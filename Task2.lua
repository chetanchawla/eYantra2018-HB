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

    green=sim.getObjectHandle('Orientation_hoop1')
    yellow=sim.getObjectHandle('Orientation_hoop2')
    red=sim.getObjectHandle('Orientation_hoop3')
    cashew=sim.getObjectHandle('Position_hoop3')
    mango=sim.getObjectHandle('Position_hoop2')
    sal=sim.getObjectHandle('Position_hoop1')
    nofruit=sim.getObjectHandle('obstacle_1')

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
    len = #msg.markers
    for i=1,len do
        ids[i]=msg.markers[i].id
        --val={msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,-msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w}
        --val={msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,-msg.markers[i].pose.pose.orientation.w}
        --val={-msg.markers[i].pose.pose.orientation.x,-msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w}
        --val={msg.markers[i].pose.pose.orientation.x,-msg.markers[i].pose.pose.orientation.y,-msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w}
        val={-msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,-msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w}


        --print(val)
        --val={0,0,0,0}
        aruco[i]=val
    end
    --print(ids)
    --print(aruco)

end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    wx=msg.poses[1].position.x
    wy=msg.poses[1].position.y
    wz=msg.poses[1].position.z
    wx = wx/(-7.557)  --27.12
    wy = -wy/(-7.604)  --37.63
    --wz = 3.043-0.0827*wz -- -270.1149
    wz=2.66931-0.08738*wz
    --wz=wz/(-4)

end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    print "koi key to dabi"
    if msg.data == 1 then
        print "1 aagya"
        for j =1,#ids do
            if ids[j] == 0 then
                print "set green"
                sim.setObjectQuaternion(green,-1,aruco[j])
                print(aruco[j])
            elseif ids[j] == 1 then
                print "set yellow"
                sim.setObjectQuaternion(yellow,-1,aruco[j])
                print(aruco[j])
            elseif ids[j] == 2 then
                print "set red"
                sim.setObjectQuaternion(red,-1,aruco[j])
                print(aruco[j])
            end
        end
    elseif msg.data==2 then
        sim.setObjectPosition(sal,-1,{wx,wy,wz})
        --print wx
        --print wy
        --print wz
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg.data==3 then
        sim.setObjectPosition(mango,-1,{wx,wy,wz})
        --print wx,wy,wz
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg.data==4 then
        sim.setObjectPosition(cashew,-1,{wx,wy,wz})
        --print wx,wy,wz
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg.data==5 then
        sim.setObjectPosition(nofruit,-1,{wx,wy,wz})
        --print wx,wy,wz
        wx=2.9
        wy=0.5080
        wz=0.6779
    elseif msg.data==6 then
        wx=2.9
        wy=0.5080
        wz=0.6779
        sim.setObjectPosition(sal,-1,{wx,wy,wz})
        sim.setObjectPosition(mango,-1,{wx+1,wy,wz})
        sim.setObjectPosition(cashew,-1,{wx+2,wy,wz})
        sim.setObjectPosition(nofruit,-1,{wx+3,wy,wz})
    end
end