local SimpleTwistControl = {
    Properties = {}
}

function SimpleTwistControl:OnActivate()
	self._gravity = -9.81
	self._gotMessage = false

    self._rotate = Vector3(0,0,0)
    self._linear = Vector3(0,0,0)
    
    self.tickNotificationBus = TickBus.Connect(self);
    self.twistNotificationBus = TwistNotificationBus.Connect(self);
end

function SimpleTwistControl:TwistReceived(linear, angular)
	self._gotMessage = true
	self._linear = linear
	self._rotate = angular
end

function SimpleTwistControl:OnTick(deltaTime, currentTime)
	if not self._gotMessage then
		return
	end

	RigidBodyRequestBus.Event.SetAngularVelocity(self.entityId, self._rotate)
	
	-- Local to world linear velocity
	local worldTM = TransformBus.Event.GetWorldTM(self.entityId)
	local linearVelocityTransformed = Transform.TransformVector(worldTM, self._linear)
	
	RigidBodyRequestBus.Event.ApplyLinearImpulse(self.entityId, Vector3(0,0,self._gravity))
	RigidBodyRequestBus.Event.SetLinearVelocity(self.entityId, linearVelocityTransformed)

	self._gotMessage = false
end

function SimpleTwistControl:OnDeactivate()
    self.tickNotificationBus:Disconnect()
    self.twistNotificationBus:Disconnect()
end

return SimpleTwistControl
