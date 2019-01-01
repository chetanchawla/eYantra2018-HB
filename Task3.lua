-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here
     drone_handle = sim.getObjectHandle('eDroneBase')




    -- Subscribing to the required topic
     whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function whycon_callback(msg)
    -- Get the position of the real-world whycon marker and set the position of the drone in the simulator.
    wx=msg.poses[1].position.x
    wy=msg.poses[1].position.y
    wz=msg.poses[1].position.z
    wx = wx/(-7.557)  --27.12
    wy = -wy/(-7.604)  --37.63
    --wz = 3.043-0.0827*wz -- -270.1149
    wz=2.66931-0.08738*wz
    -- Actuation
    sim.setObjectPosition(drone_handle,-1,{wx,wy,wz})
end
