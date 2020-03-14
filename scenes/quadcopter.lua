function sysCall_init()
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v = sim.getInt32Parameter(sim.intparam_program_version)
    if (v < 20413) then
        sim.displayDialog('Warning', 'The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!', sim.dlgstyle_ok, false, '', nil, { 0.8, 0, 0, 0, 0, 0 })
    end

    -- Detatch the manipulation sphere:
    targetObj = sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj, -1, true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d = sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible = sim.getScriptSimulationParameter(sim.handle_self, 'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree, 'particlesAreVisible', tostring(particlesAreVisible))
    simulateParticles = sim.getScriptSimulationParameter(sim.handle_self, 'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', tostring(simulateParticles))

    propellerScripts = { -1, -1, -1, -1 }
    for i = 1, 4, 1 do
        propellerScripts[i] = sim.getScriptHandle('Quadricopter_propeller_respondable' .. i)
    end
    heli = sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities = { 0, 0, 0, 0 }

    pParam = 2
    iParam = 0
    dParam = 0
    vParam = -2

    cumul = 0
    lastE = 0
    pAlphaE = 0
    pBetaE = 0
    psp2 = 0
    psp1 = 0

    prevEuler = 0

    fakeShadow = sim.getScriptSimulationParameter(sim.handle_self, 'fakeShadow')
    if (fakeShadow) then
        shadowCont = sim.addDrawingObject(sim.drawing_discpoints + sim.drawing_cyclic + sim.drawing_25percenttransparency + sim.drawing_50percenttransparency + sim.drawing_itemsizes, 0.2, 0, -1, 1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam = sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam = sim.getObjectHandle('Quadricopter_frontCamera')
    floorView = sim.floatingViewAdd(0.9, 0.9, 0.2, 0.2, 0)
    frontView = sim.floatingViewAdd(0.7, 0.9, 0.2, 0.2, 0)
    sim.adjustView(floorView, floorCam, 64)
    sim.adjustView(frontView, frontCam, 64)


    --
    visionSensor_h = sim.getObjectHandle('Vision_sensor')
    imu_handle = sim.getObjectHandle('IMU_link')

    if simROS then

        revsSub = simROS.subscribe('/rotorRevs', 'std_msgs/Float64MultiArray', 'rotorRevs_cb')


        Gyro_pub = simROS.advertise('/imu', 'sensor_msgs/Imu')
        simROS.publisherTreatUInt8ArrayAsString(Gyro_pub)
        target_pub = simROS.advertise('/target', 'geometry_msgs/Pose')
        simROS.publisherTreatUInt8ArrayAsString(target_pub)
        Odom_pub = simROS.advertise('/odom','nav_msgs/Odometry')
        simROS.publisherTreatUInt8ArrayAsString(Odom_pub)

        Imu_data = {}
        gyroCommunicationTube = sim.tubeOpen(0, 'gyroData' .. sim.getNameSuffix(nil), 1)
        accelCommunicationTube = sim.tubeOpen(0, 'accelerometerData' .. sim.getNameSuffix(nil), 1)
    end

    rotorRevs={0,0,0,0}

    initTime = sim.getSimulationTime()

end


function rotorRevs_cb(msg)

    rotorRevs[1] = msg.data[1]*3.3
    rotorRevs[2] = msg.data[2]*3.3
    rotorRevs[3] = msg.data[3]*3.3
    rotorRevs[4] = msg.data[4]*3.3

end

function sysCall_cleanup()
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end

function sysCall_actuation()
    s = sim.getObjectSizeFactor(d)

    pos = sim.getObjectPosition(d, -1)
    if (fakeShadow) then
        itemData = { pos[1], pos[2], 0.002, 0, 0, 1, 0.2 * s }
        sim.addDrawingObjectItem(shadowCont, itemData)
    end


    -- Vertical control:
    targetPos = sim.getObjectPosition(targetObj, -1)
    pos = sim.getObjectPosition(d, -1)
    l = sim.getVelocity(heli)
    e = (targetPos[3] - pos[3])
    cumul = cumul + e
    pv = pParam * e
    --5.335=kmg->k=5.335/1/9.8=0.544387755
    thrust = 5.335 + pv + iParam * cumul + dParam * (e - lastE) + l[3] * vParam --- 0.544387755*1*errorZ
    lastE = e

    -- Horizontal control:
    sp = sim.getObjectPosition(targetObj, d)
    m = sim.getObjectMatrix(d, -1)
    vx = { 1, 0, 0 }
    vx = sim.multiplyVector(m, vx)
    vy = { 0, 1, 0 }
    vy = sim.multiplyVector(m, vy)
    --m[12]->z height
    alphaE = (vy[3] - m[12])
    alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - pAlphaE)
    betaE = (vx[3] - m[12])
    betaCorr = -0.25 * betaE - 2.1 * (betaE - pBetaE)
    pAlphaE = alphaE
    pBetaE = betaE

    alphaCorr = alphaCorr + sp[2] * 0.005 + 1 * (sp[2] - psp2) -- errorY/30
    betaCorr = betaCorr - sp[1] * 0.005 - 1 * (sp[1] - psp1) -- errorX/30
    psp2 = sp[2]
    psp1 = sp[1]

    -- Rotational control:
    euler = sim.getObjectOrientation(d, targetObj)
    rotCorr = euler[3] * 0.1 + 2 * (euler[3] - prevEuler)
    prevEuler = euler[3]

    -- Decide of the motor velocities:
    --print(rotorRevs)
    if (rotorRevs[1] == 0 or (sim.getSimulationTime()-initTime)<2) then
        particlesTargetVelocities[1] = thrust * (1 - alphaCorr + betaCorr + rotCorr)--+error2
        particlesTargetVelocities[2] = thrust * (1 - alphaCorr - betaCorr - rotCorr)--+error1
        particlesTargetVelocities[3] = thrust * (1 + alphaCorr - betaCorr + rotCorr)--+error4
        particlesTargetVelocities[4] = thrust * (1 + alphaCorr + betaCorr - rotCorr)--+error3
    else
        particlesTargetVelocities[1] = rotorRevs[1]
        particlesTargetVelocities[2] = rotorRevs[2]
        particlesTargetVelocities[3] = rotorRevs[3]
        particlesTargetVelocities[4] = rotorRevs[4]
    end
    --print(particlesTargetVelocities)
    -- Send the desired motor velocities to the 4 rotors:
    for i = 1, 4, 1 do
        sim.setScriptSimulationParameter(propellerScripts[i], 'particleVelocity', particlesTargetVelocities[i])
    end

end

function sysCall_sensing()
    if simROS then
        quaternion = sim.getObjectQuaternion(imu_handle, -1)
        accele_data = sim.tubeRead(accelCommunicationTube)
        gyro_data = sim.tubeRead(gyroCommunicationTube)
        if (accele_data and gyro_data) then
            acceleration = sim.unpackFloatTable(accele_data)
            angularVariations = sim.unpackFloatTable(gyro_data)
            Imu_data['orientation'] = { x = quaternion[1], y = quaternion[2], z = quaternion[3], w = quaternion[4] }
            Imu_data['header'] = { seq = 0, stamp = sim.getSystemTime(), frame_id = "imu_link" }
            Imu_data['linear_acceleration'] = { x = acceleration[1], y = acceleration[2], z = -acceleration[3] }
            Imu_data['angular_velocity'] = { x = angularVariations[1], y = angularVariations[2], z = angularVariations[3] }

            --simROS.sendTransform(getTransformStamped(imu_handle,'imu_link',base_handle,'base_link'))
            simROS.publish(Gyro_pub, Imu_data)

            local pos = sim.getObjectPosition(imu_handle,  -1)
            local ori = sim.getObjectQuaternion(imu_handle, -1)
            local vel = sim.getObjectVelocity(imu_handle, -1)

            odom = {}
            odom.header = {seq=0,stamp=sim.getSystemTime(), frame_id="odom"}
            odom.child_frame_id = 'base_link'
            odom.pose = { pose={position={x=pos[1],y=pos[2],z=pos[3]}, orientation={x=ori[1],y=ori[2],z=ori[3],w=ori[4]} } }
            odom.twist={twist = { linear={x=vel[1], y=vel[2], z=vel[3]},angular = { x = angularVariations[1], y = angularVariations[2], z = angularVariations[3] }}}

            simROS.publish(Odom_pub, odom)
        end

        local targetPos = sim.getObjectPosition(targetObj, -1)
        local targetOri = sim.getObjectQuaternion(targetObj, -1)
        target_data={}
        target_data['position']={x=targetPos[1],y=targetPos[2],z=targetPos[3]}
        target_data['orientation']={x=targetOri[1],y=targetOri[2],z=targetOri[3],w=targetOri[4]}
        simROS.publish(target_pub, target_data)
    end
end