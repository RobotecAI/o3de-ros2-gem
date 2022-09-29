

--[[
    This is a proof-of-concept implementation of vehicle control physics. 
    It allows controlling the speed and steering angle of the vehicle 
    using the keyboard.
--]]


function clamp(value, lower_limit, upper_limit)
    if lower_limit == upper_limit then
        return value
    else
        if value > upper_limit then
            return upper_limit
        elseif value < lower_limit then
            return lower_limit
        else
            return value
        end
    end
end

local PID = {}
PID.__index = PID

function PID.new(kp, kd, ki, limit_output)
    local self = setmetatable({}, PID)
    self._kp = kp
    self._kd = kd
    self._ki = ki
    self._preError = 0
    self._integral = 0
    self._limit = limit_output
    return self
end


function PID:Reset()
    self._preError = 0
    self._integral = 0
end


function PID:Calculate(dt, setpoint, pv)
    local err = (setpoint - pv)
    local pOut = (self._kp * err)
    self._integral = self._integral + (err * dt)
    local iOut = (self._ki * self._integral)
    local deriv = ((err - self._preError) / dt)
    local dOut = (self._kd * deriv)
    output = (pOut + iOut + dOut)
    self._preError = err
    if self._limit>0.0 then
        output = clamp(output, -self._limit, self._limit)
    end
    return output
end




local manipulator_control = 
{
    Properties =
    {
    -- Entities that are needed by the control

            segment1 = { default = EntityId() },
            segment2 = { default = EntityId() },
            segment3 = { default = EntityId() },
            segment4 = { default = EntityId() },



    }
}




function manipulator_control:OnActivate()
     -- Activation Code
     
     Debug.Log("-- Joint test initialization")

     self.tickBusHandler = TickBus.CreateHandler(self)
     self.tickBusHandler:Connect()    
     
     self.InputNotificationBus = InputEventNotificationBus.Connect(self, InputEventNotificationId("manipulator_keyboard_control"))


     self.target_position = {0.0, 0.0, 0.0, 0.0}


     --self.pid1 = PID.new(2000.0, 300.0, 200.0)
     --self.pid2 = PID.new(200.0, 100.0, 100.0)
     --self.pid3 = PID.new(1000.0, 0.0, 0.0)

     --self.pid1 = PID.new(2000.0, 300.0, 200.0)
     --self.pid2 = PID.new(200.0, 100.0, 100.0)
     --self.pid3 = PID.new(0.0, 0.0, 0.0)

     self.pid1 = PID.new(10.0, 1.0, 0.0, 0.0) 
     self.pid2 = PID.new(8.0, 1.0, 0.0, 0.0) 
     self.pid3 = PID.new(0.0, 0.0, 0.0, 0.0)
     self.pid4 = PID.new(20.0, 0.0, 0.0, 0.0)

     --self.segment1_limits = {-0.9, 0.9}
     --self.segment2_limits = {-0.7, 0.7}
     self.segment1_limits = {-0.6, 0.6}
     self.segment2_limits = {-0.35, 0.35}
     self.segment3_limits = {-0.10, 0.10}
     self.segment4_limits = {-0.10, 0.10}

     self.zeroPose = {nil, nil, nil, nil}

     -- To prevent violent reactions right after the simulation starts,
     -- we're waiting this ammount of seconds till running the controller
     self.startup_wait = 4.0 --[s]

     self.twistNotificationBus = JoyNotificationBus.Connect(self);

end


function manipulator_control:getSegmentPos(entityid)
    return TransformBus.Event.GetLocalTranslation(entityid)
end

function manipulator_control:JoyReceived(a0, a1, a2, a3, a4, a5, b0, b1, b2, b3, b4, b5, b6, b7 )
    Debug.Log("["..string.format("%1.4f",a0).."]")
    self.target_position[1] = self.target_position[1] - 0.1 * a3 * self.segment1_limits[1]
    self.target_position[2] = self.target_position[2] - 0.1 * a2 * self.segment1_limits[1]
    self.target_position[4] = self.target_position[2] - 0.1 * a5 * self.segment1_limits[1]

end


function manipulator_control:setSegmentPos(entityid, pid, segment, output_direction, axis1, print_debug)

    --axis = 'z'


    local target_pos = self.target_position[segment]
    --local current_pos = TransformBus.Event.GetLocalTranslation(entityid)[axis]
    local current_pos = self:getSegmentPos(entityid)-self.zeroPose[segment]
    current_pos = current_pos[axis1]

    --Debug.Log('is: '..string.format("%1.2f",current_pos)..'  should be: '..string.format("%1.2f",target_pos))

    local force = pid:Calculate(self.deltaTime, target_pos, current_pos)
    local impulse = force * self.deltaTime



    local tm = TransformBus.Event.GetWorldTM(entityid)

    local force_vector = Vector3(0.0, 0.0, 0.0)
    local velocity_vector = Vector3(0.0, 0.0, 0.0)

    force_vector["z"] = impulse * output_direction
    force_vector = Transform.TransformVector(tm, force_vector)

    velocity_vector["z"] = force * output_direction
    velocity_vector = Transform.TransformVector(tm, velocity_vector)

    --RigidBodyRequestBus.Event.ApplyLinearImpulse(entityid, force_vector)   
    RigidBodyRequestBus.Event.SetLinearVelocity(entityid, velocity_vector)   
    --if entityid == self.Properties.segment2 then
        -- local entity_name = GameEntityContextRequestBus.Event.GetEntityName(entityid) --TODO make this work
    if print_debug then
        Debug.Log('[ '..tostring(entityid)..'] is: '..string.format("%1.3f",current_pos)..'  should be: '..string.format("%1.3f",target_pos)..'  impulse: '..string.format("%1.3f",force))
    end


end

function manipulator_control:setPosition()
    self:setSegmentPos(self.Properties.segment1, self.pid1, 1, 1, 'z', false)
    self:setSegmentPos(self.Properties.segment2, self.pid2, 2, 1, 'x', false)
    self:setSegmentPos(self.Properties.segment3, self.pid3, 3, -1, 'y', true)
    self:setSegmentPos(self.Properties.segment4, self.pid4, 4, 1, 'z', true)

end

function manipulator_control:OnHeld  (value)
    -- Keypress actions

    -- Process key press

    -- UP --
    if value == 8.0 then
        --Debug.Log('up')
        --self.target_position[1] = 0.7
        self.target_position[1] = self.segment1_limits[2]

    end
    -- DOWN --
    if value == 2.0 then
        --Debug.Log('down')

        --self.target_position[1] = -0.7
        self.target_position[1] = self.segment1_limits[1]

    end
    -- LEFT --
    if value == 4.0 then
        --self.target_position[2] = 0.35
        self.target_position[2] = self.segment2_limits[1]


    end
    -- RIGHT --
    if value == 6.0 then
        --self.target_position[2] = -0.35
        self.target_position[2] = self.segment2_limits[2]

    end

    if value == 5.0 then
        self.target_position[1] = 0.0
        self.target_position[2] = 0.0
        self.target_position[3] = 0.0
        self.target_position[4] = 0.0

    end


    if value == 7.0 then
        self.target_position[3] = self.segment3_limits[2]
        self.target_position[4] = self.segment4_limits[1]

    end

    if value == 9.0 then
        self.target_position[3] = self.segment3_limits[1]
        self.target_position[4] = self.segment4_limits[2]
    end
end

function manipulator_control:OnDeactivate()
     -- Deactivation Code
end

-- This callback is called every frame by the tick bus after this entity activates
function manipulator_control:OnTick(deltaTime, timePoint)

    self.deltaTime = deltaTime

    if self.Properties.segment1~=nil then
        if self.zeroPose[1]==nil then
            self.zeroPose = {
                self:getSegmentPos(self.Properties.segment1),
                self:getSegmentPos(self.Properties.segment2),
                self:getSegmentPos(self.Properties.segment3),
                self:getSegmentPos(self.Properties.segment4),
            }

            local txt = " ZERO POSE: "
            for i=1,#self.zeroPose do
                txt = txt.." #"..tostring(i)..tostring(self.zeroPose[i])
            end
            Debug.Log(txt)
        else
            if self.startup_wait > 0 then
                self.startup_wait = self.startup_wait -self.deltaTime
                --Debug.Log(string.format("%1.3f",self.startup_wait))
            else
                self:setPosition()
            end
        end
    end


end



return manipulator_control