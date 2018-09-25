--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2018 Peter Vajko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
]]

--[[

This is a simplified implementation of a pure pursuit algorithm
to follow a two dimensional path consisting of waypoints.

See the paper

Steering Control of an Autonomous Ground Vehicle with Application to the DARPA
Urban Challenge By Stefan F. Campbell

We use the terminology of that paper here, like 'relevant path segment', 'goal point', etc.

]]

PurePursuitController = {}
PurePursuitController.__index = PurePursuitController

-- constructor
function PurePursuitController:new(vehicle)
	local newPpc = {}
	setmetatable( newPpc, self )
	newPpc.lookAheadDistance = 5
	-- when transitioning from forward to reverse, this close we have to be to the waypoint where we
	-- change direction before we switch to the next waypoint
	newPpc.distToSwitchWhenChangingToReverse = 1
	newPpc.vehicle = vehicle
	newPpc.name = nameNum(vehicle)
	-- node on the current waypoint
	newPpc.currentWpNode = WaypointNode:new( newPpc.name .. '-currentWpNode', vehicle, true)
	-- waypoint at the start of the relevant segment
	newPpc.relevantWpNode = WaypointNode:new( newPpc.name .. '-relevantWpNode', vehicle, true)
	-- waypoint at the end of the relevant segment
	newPpc.nextWpNode = WaypointNode:new( newPpc.name .. '-nextWpNode', vehicle, true)
	-- the current goal node
	newPpc.goalWpNode = WaypointNode:new( newPpc.name .. '-goalWpNode', vehicle, false)
	-- vehicle position projected on the path, not used for anything other than debug display
	newPpc.projectedPosNode = courseplay.createNode( newPpc.name .. '-projectedPosNode', 0, 0, 0)
	return newPpc
end

-- destructor
function PurePursuitController:delete()
	self.currentWpNode:destroy()
	self.relevantWpNode:destroy()
	self.nextWpNode:destroy()
	courseplay.destroyNode(self.projectedPosNode)
	self.goalWpNode:destroy();
end

-- initialize controller before driving
function PurePursuitController:initialize()
--[[	self.segments = {}
	local nSegments = 1

	for i = self.vehicle.cp.waypointIndex, #self.vehicle.Waypoints do
		if self:switchingDirectionAt(i) and self.segments[nSegments] then
			-- start a new segment here (unless this is the first wp)
			nSegments = nSegments + 1
		end
		if not self.segments[nSegments] then
			self.segments[nSegments] = {}
		end
		table.insert(self.segments[nSegments], Waypoint:new(self.vehicle.Waypoints[i], i))
	end
	self.currentSegment = 1
	--]]
	-- we rely on the code in start_stop.lua to select the first waypoint
	-- relevantWpNode always points to the point where the relevant path segment starts
	self.currentWpNode:setToWaypoint(self.vehicle.cp.waypointIndex)
	self.relevantWpNode:setToWaypoint(self.vehicle.cp.waypointIndex)
	self.nextWpNode:setToWaypoint(self.vehicle.cp.waypointIndex + 1)
	courseplay.debugVehicle(12, self.vehicle, 'PPC: initialized to waypoint %d', self.vehicle.cp.waypointIndex)
	self.enabled = true
end

function PurePursuitController:getCurrentWaypointIx()
	return self.currentWpNode.ix
end

-- distance between (px,pz) and the ix waypoint
function PurePursuitController:getDistanceToWaypoint(px, pz, ix)
	local x, z = self.vehicle.Waypoints[ix].cx, self.vehicle.Waypoints[ix].cz
	return courseplay:distance(px, pz, x, z)
end



function PurePursuitController:update()
	self:findRelevantSegment()
	self:findGoalPoint()
	self:deactivateWhenInReverse()
end

function PurePursuitController:havePassedNextWaypoint(wpNode)
	local vx, vy, vz = getWorldTranslation(self.vehicle.cp.DirectionNode or self.vehicle.rootNode)
	local dx, _, dz = worldToLocal(wpNode.node, vx, vy, vz);
	local dFromNext = Utils.vector2Length(dx, dz)

	if Waypoint.switchingToReverseAt(self.vehicle, wpNode.ix) or
		Waypoint.switchingToForwardAt(self.vehicle, wpNode.ix) then
		-- switching direction at this waypoint, so this is pointing into the opposite direction.
		-- we have to make sure we drive up to this waypoint close enough before we switch to the next
		-- so wait until dz < 0, that is, we are behind the waypoint 
		if dz < 0 then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: waypoint %d passed, before switching direction, dz: %.1f', wpNode.ix, dz)
			return true
		end
	else
		-- we are not transitioning between forward and reverse
		local reversed = ''
		if self.vehicle.Waypoints[wpNode.ix].rev then
			-- when reversing, we must be _behind_ (dz < 0) the waypoint to pass it
			-- as the vehicle's direction node still points to forward
			dz = -dz
			reversed = ' (reversed)'
		end
		-- have we passed the next waypoint? Must get closer than lookahead distance to switch to make sure we
		-- actually drive back to the waypoint even if we are already ahead of it
		if dz >= 0 and dFromNext < self.lookAheadDistance then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: waypoint %d passed, dz: %.1f %s', wpNode.ix, dz, reversed)
			return true
		end
	end
	
	return false
end

-- Finds the relevant segment.
-- Sets the vehicle's projected position on the path.
function PurePursuitController:findRelevantSegment()
	-- vehicle position
	local vx, vy, vz = getWorldTranslation(self.vehicle.cp.DirectionNode or self.vehicle.rootNode)
	local crossTrackError, _, dzFromRelevant = worldToLocal(self.relevantWpNode.node, vx, vy, vz);
	-- projected vehicle position/rotation
	local px, py, pz = localToWorld(self.relevantWpNode.node, 0, 0, dzFromRelevant)
	local _, yRot, _ = getRotation(self.nextWpNode.node)
	-- have we passed the next waypoint? Must get closer than lookahead distance to switch to make sure we
	-- actually drive back to the waypoint even if we are already ahead of it
	if self:havePassedNextWaypoint(self.nextWpNode) then
		self.relevantWpNode:setToWaypoint(self.relevantWpNode.ix + 1)
		self.nextWpNode:setToWaypoint(self.relevantWpNode.ix + 1, self.lookAheadDistance)
		courseplay.debugVehicle(12, self.vehicle, 'PPC: relevant waypoint: %d, crosstrack error: %.1f', self.relevantWpNode.ix, crossTrackError)
	end
	setTranslation(self.projectedPosNode, px, py, pz)
	setRotation(self.projectedPosNode, 0, yRot, 0)
	if courseplay.debugChannels[12] then
		drawDebugLine(px, py + 3, pz, 1, 1, 0, px, py + 1, pz, 1, 1, 0);
		DebugUtil.drawDebugNode(self.relevantWpNode.node, string.format('ix = %d\nrelevant\nnode', self.relevantWpNode.ix, dz))
		DebugUtil.drawDebugNode(self.projectedPosNode, 'projected\nvehicle\nposition')
	end
end

-- Now, from the relevant section forward we search for the goal point, which is the one
-- lying lookAheadDistance in front of us on the path
function PurePursuitController:findGoalPoint()
	local d1, d2
	local tx, ty, tz

	local vx, vy, vz = getWorldTranslation(self.vehicle.cp.DirectionNode or self.vehicle.rootNode);

	-- create helper nodes at the relevant and the next wp. We'll move these up on the path until we reach the segment
	-- in lookAheadDistance
	local node1 = WaypointNode:new( self.name .. '-node1', self.vehicle, false)
	local node2 = WaypointNode:new( self.name .. '-node2', self.vehicle, false)

	local isGoalPointValid = false

	-- starting at the relevant segment walk up the path to find the segment on
	-- which the goal point lies. This is the segment intersected by the circle with lookAheadDistance radius
	-- around the vehicle.
	local ix = self.relevantWpNode.ix
	courseplay.debugVehicle(12, self.vehicle, 'relevant Ix: %d', self.relevantWpNode.ix) -- -----------------------
	while ix <= #self.vehicle.Waypoints do
		courseplay.debugVehicle(12, self.vehicle, 'ix: %d', ix) -- -----------------------
		node1:setToWaypoint(ix)
		node2:setToWaypointOrBeyond(ix + 1, self.lookAheadDistance)
		local x1, y1, z1 = getWorldTranslation(node1.node)
		local x2, y2, z2 = getWorldTranslation(node2.node)
		-- distance between the vehicle position and the end of the segment
		d1 = courseplay:distance(vx, vz, x2, z2)
		courseplay.debugVehicle(12, self.vehicle, 'ix: %d %.4f', ix, d1) -- -----------------------
		if d1 > self.lookAheadDistance then
			-- far end of this segment is farther than lookAheadDistance so the goal point must be on
			-- this segment
			d2 = courseplay:distance(x1, z1, vx, vz)
			courseplay.debugVehicle(12, self.vehicle, 'ix: %d dFromPrev: %.4f dToNext: %.4f', ix, d2, d1) -- -----------------------
			if d2 > self.lookAheadDistance then
				-- too far from either end of the relevant segment
				if not self.isGoalPointValid then
					-- If we weren't on track yet (after initialization, on our way to the first/initialized waypoint)
					-- set the goal to the relevant WP
					self.goalWpNode:setToWaypoint(self.relevantWpNode.ix)
					-- and also the current waypoint is now at the relevant WP
					self.currentWpNode:setToWaypointOrBeyond(self.relevantWpNode.ix, self.lookAheadDistance)
					DebugUtil.drawDebugNode(self.goalWpNode.node, string.format('\n\n\ntoo far\ninitializing'))
					break
				else
					-- we already were tracking the path but now both points are too far. 
					-- we can go ahead and find the goal point as usual, as we start approximating
					-- from the front waypoint and will find the goal point in front of us.
					-- isGoalPointValid = true
					DebugUtil.drawDebugNode(self.goalWpNode.node, string.format('\n\n\ntoo far'))
				end
			end
			-- our goal point is now between ix and ix + 1, let's find it
			-- distance between current and next waypoint
			local dToNext = courseplay:distance(x1, z1, x2, z2)
			local minDz, maxDz, currentDz, currentRange = 0, dToNext, dToNext / 2, dToNext
			-- this is the current waypoint for the rest of Courseplay code, the waypoint we are driving to
			self.currentWpNode:setToWaypointOrBeyond(ix + 1, self.lookAheadDistance)

			
			-- successive approximation of the intersection between this path segment and the
			-- lookAheadDistance radius circle around the vehicle. That intersection point will be our goal point

			-- starting from the far end makes sure we find the correct point even in the case when the
			-- circle around the vehicle intersects with this section twice.
			
			local bits = 8  -- successive approximator (ADC) bits
			local step = 0  -- current step
			local epsilon = self.lookAheadDistance / (math.pow(2, bits - 1)) -- accuracy of our ADC
			while step < bits do
				-- pint in currentDz distance from node1 on the section between node1 and node2
				local gx, gy, gz = localToWorld(node1.node, 0, 0, currentDz)
				d1 = courseplay:distance(vx, vz, gx, gz)
				--courseplay.debugVehicle(12, self.vehicle, 'range: %.4f d1: %.4f, dz: %.4f', currentRange, d1, currentDz) -- -----------------------------
				if d1 < self.lookAheadDistance + epsilon and d1 > self.lookAheadDistance - epsilon then
					-- we are close enough to the goal point.
					gy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, gx, 0, gz)
					setTranslation(self.goalWpNode.node, gx, gy, gz)
					isGoalPointValid = true
					tx, ty, tz = localToWorld(self.goalWpNode.node, 0, 0, 0)
					if courseplay.debugChannels[12] then
						DebugUtil.drawDebugNode(self.goalWpNode.node, string.format('ix = %d\nd = %.1f\ncr = %.1f\ngoal\npoint', ix, d1, currentRange))
						DebugUtil.drawDebugNode(self.currentWpNode.node, string.format('ix = %d\ncurrent\nwaypoint', self.currentWpNode.ix))
						drawDebugLine(gx, gy + 3, gz, 0, 1, 0, gx, gy + 1, gz, 0, 1, 0);
					end
					break
				end
				if d1 < self.lookAheadDistance then
					minDz = currentDz
				else
					maxDz = currentDz
				end
				step = step + 1
				currentRange = currentRange / 2
				currentDz = minDz + currentRange
			end
			break
		end
		ix = ix + 1
	end
	courseplay.destroyNode(node1.node)
	courseplay.destroyNode(node2.node)
	self:setGoalPointValid(isGoalPointValid)
	return tx, ty, tz
end

function PurePursuitController:setGoalPointValid(isGoalPointValid)
	if self.isGoalPointValid ~= isGoalPointValid then
		if isGoalPointValid then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: Goal point found.')
		else
			courseplay.debugVehicle(12, self.vehicle, 'PPC: Goal point lost.')
		end
		self.isGoalPointValid = isGoalPointValid
	end
end


-- Let the code in reverse.lua do its magic when reversing.
-- That code seems to be robust against circling anyway
function PurePursuitController:deactivateWhenInReverse()
	if self.vehicle.Waypoints[self.currentWpNode.ix].rev then
		if self.enabled then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: waypoint %d is reverse, deactivate PPC.', self.currentWpNode.ix)
		end
		self.enabled = false
	else
		if not self.enabled then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: waypoint %d is not reverse, (re)activate PPC.', self.currentWpNode.ix)
		end
		self.enabled = true
	end
end

function PurePursuitController:getDirection(lz)

	local ctx, cty, ctz = self:getClosestWaypointData()
	if not ctx then return lz end
	local dx, _, dz  = worldToLocal(self.vehicle.cp.DirectionNode, ctx, cty, ctz)
	local x, _, z = localToWorld(self.vehicle.cp.DirectionNode, 0, 0, 0)
	local distance = math.sqrt(dx * dx + dz * dz)
	local r = distance * distance / 2 / dx
	local steeringAngle = math.atan(self.vehicle.cp.distances.frontWheelToRearWheel / r)
	return math.cos(steeringAngle)
end

function PurePursuitController:isActive()
	-- for now, turn on/off PPC with debug channel 7.
	return courseplay.debugChannels[7] and self.enabled
end

function PurePursuitController:getCurrentWaypointPosition()
	local cx, cz
	if self:isActive() then
		cx, _, cz = getWorldTranslation(self.currentWpNode.node)
	else
		cx, cz = self.vehicle.Waypoints[self.vehicle.cp.waypointIndex].cx, self.vehicle.Waypoints[self.vehicle.cp.waypointIndex].cz
	end
	return cx, cz
end

function PurePursuitController:switchToNextWaypoint()
	if self:isActive() then
		courseplay:setWaypointIndex(self.vehicle, self:getCurrentWaypointIx());
	else
		courseplay:setWaypointIndex(self.vehicle, self.vehicle.cp.waypointIndex + 1);
	end
end

-- This is to be used in drive.lua in place of the dist < distToChange check, that is, when we
-- reached the next waypoint.
function PurePursuitController:shouldChangeWaypoint(distToChange)
	local shouldChangeWaypoint
	if self:isActive() then
		-- true when the current waypoint calculated by PPC does not match the CP waypoint anymore, or
		-- true when at the last waypoint (to trigger the last waypoint processing in drive.lua (which was triggerd by
		-- the distToChange condition before PPC)
		shouldChangeWaypoint = self:getCurrentWaypointIx() ~= self.vehicle.cp.waypointIndex
	else
		shouldChangeWaypoint = self.vehicle.cp.distanceToTarget <= distToChange
	end
	return shouldChangeWaypoint
end

function PurePursuitController:atLastWaypoint()
	local atLastWaypoint = not (self.vehicle.cp.waypointIndex < self.vehicle.cp.numWaypoints)
	return atLastWaypoint
end
