function handleUI(p)
    local s=sim.getObjectSelection()
    if s and #s>0 and s[#s]==model then
        if not ui then
            local xml =[[<ui title="Gyro sensor" closeable="false" placement="relative" position="50,-50" layout="form">
                    <label text="x gyro:" />
                    <label id="1" text="-" />
                    <label text="y gyro:" />
                    <label id="2" text="-" />
                    <label text="z gyro:" />
                    <label id="3" text="-" />
            </ui>]]
            ui=simUI.create(xml)
        end
        simUI.setLabelText(ui,1,string.format("%.5f",p[1]))
        simUI.setLabelText(ui,2,string.format("%.5f",p[2]))
        simUI.setLabelText(ui,3,string.format("%.5f",p[3]))
    else
        if ui then
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function sysCall_init() 
    base= sim.getObject('/MyRobot')
    model=sim.getObject('.')
    ref=sim.getObject('./reference')
    oldTransformationMatrix=sim.getObjectMatrix(ref,-1)
    lastTime=sim.getSimulationTime()
end

function sysCall_sensing() 
    local transformationMatrix=sim.getObjectMatrix(ref,-1)
    local oldInverse=sim.copyTable(oldTransformationMatrix)
    sim.invertMatrix(oldInverse)
    local m=sim.multiplyMatrices(oldInverse,transformationMatrix)
    local euler=sim.getEulerAnglesFromMatrix(m)
    local currentTime=sim.getSimulationTime()
    local gyroData={0,0,0}
   -- local gData = 0;
    local dt=currentTime-lastTime
    if (dt~=0) then
        --gData = euler[1]/dt
        gyroData[1]=euler[1]/dt
        gyroData[2]=euler[2]/dt
        gyroData[3]=euler[3]/dt
    end
    oldTransformationMatrix=sim.copyTable(transformationMatrix)
    lastTime=currentTime
    sim.writeCustomDataBlock(base, 'g_data', sim.packTable(gyroData))
    sim.writeCustomDataBlock(base, 'dt', sim.packTable({dt}))

    handleUI(gyroData)
end 
