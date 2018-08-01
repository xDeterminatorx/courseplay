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
	newPpc.vehicle = vehicle
	newPpc.name = nameNum(vehicle)
	-- node on the current waypoint
	newPpc.currentWpNode = courseplay.createNode( newPpc.name .. 'currentWpNode', 0, 0, 0)
	-- waypoint at the start of the relevant segment
	newPpc.relevantWpNode = courseplay.createNode( newPpc.name .. 'relevantWpNode', 0, 0, 0)
	-- waypoint at the end of the relevant segment
	newPpc.nextWpNode = courseplay.createNode( newPpc.name .. 'nextWpNode', 0, 0, 0)
	-- vehicle position projected on the path
	newPpc.projectedPosNode = courseplay.createNode( newPpc.name .. 'projectedPosNode', 0, 0, 0)
	-- the current goal node
	newPpc.goalNode = courseplay.createNode( newPpc.name .. 'goalNode', 0, 0, 0)
	return newPpc
end

-- destructor
function PurePursuitController:delete()
	courseplay.destroyNode(self.currentWpNode)
	courseplay.destroyNode(self.relevantWpNode)
	courseplay.destroyNode(self.nextWpNode)
	courseplay.destroyNode(self.projectedPosNode)
	courseplay.destroyNode(self.goalNode)
end

-- initialize controller before driving
function PurePursuitController:initialize()
	-- we rely on the code in start_stop.lua to select the first waypoint
	-- relevantIx always points to the point where the relevant path segment starts
	self.relevantIx = self.vehicle.cp.waypointIndex
	self.currentIx = self.vehicle.cp.waypointIndex
	self:setNodeToWaypoint(self.currentWpNode, self.relevantIx)
	self:setNodeToWaypoint(self.relevantWpNode, self.relevantIx)
	self:setNodeToWaypoint(self.nextWpNode, self.relevantIx)
	courseplay.debugVehicle(12, self.vehicle, 'PPC: initialized to waypoint %d', self.relevantIx)
end

function PurePursuitController:getCurrentWaypointIx()
	return self.currentIx
end

-- distance between (px,pz) and the ix waypoint
function PurePursuitController:getDistanceToWaypoint(px, pz, ix)
	local x, z = self.vehicle.Waypoints[ix].cx, self.vehicle.Waypoints[ix].cz
	return courseplay:distance(px, pz, x, z)
end

function PurePursuitController:setNodeToWaypoint(node, ix)
	local x, y, z = self:getWaypointPosition(ix)
	setTranslation(node, x, y, z)
	setRotation(node, 0, math.rad(self.vehicle.Waypoints[ix].angle), 0)
end

function PurePursuitController:getWaypointPosition(ix)
	local x, z = self.vehicle.Waypoints[ix].cx, self.vehicle.Waypoints[ix].cz
	local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
	return x, y, z
end

function PurePursuitController:update()
	self:findRelevantSegment()
	self:findGoalPoint()
end

-- Finds the relevant segment.
-- Sets the vehicle's projected position on the path.
function PurePursuitController:findRelevantSegment()
	-- vehicle position
	local vx, vy, vz = getWorldTranslation(self.vehicle.cp.DirectionNode or self.vehicle.rootNode)
	local crossTrackError, _, dzFromRelevant = worldToLocal(self.relevantWpNode, vx, vy, vz);
	local dx, _, dz = worldToLocal(self.nextWpNode, vx, vy, vz);
	local dFromNext = Utils.vector2Length(dx, dz)
	-- projected vehicle position/rotation
	local px, py, pz = localToWorld(self.relevantWpNode, 0, 0, dzFromRelevant)
	local _, yRot, _ = getRotation(self.nextWpNode)
	-- have we passed the next waypoint? Must get closer than lookahead distance to switch to make sure we
	-- actually drive back to the waypoint even if we are already ahead of it
	if dz >= 0 and dFromNext < self.lookAheadDistance and self.relevantIx < #self.vehicle.Waypoints then
		self.relevantIx = self.relevantIx + 1
		courseplay.debugVehicle(12, self.vehicle, 'PPC: relevant waypoint: %d, crosstrack error: %.1f', self.relevantIx, crossTrackError)
		self:setNodeToWaypoint(self.relevantWpNode, self.relevantIx)
		if self.relevantIx < #self.vehicle.Waypoints then
			self:setNodeToWaypoint(self.nextWpNode, self.relevantIx + 1)
		else
			-- we are already at the last waypoint, there's no next. So put it beyond the last one
			local nx, ny, nz = localToWorld(self.nextWpNode, 0, 0, self.lookAheadDistance)
			setTranslation(self.nextWpNode, nx, ny, nz)
			courseplay.debugVehicle(12, self.vehicle, 'PPC: last waypoint reached, moving next waypoint node beyond last')
		end
	end
	setTranslation(self.projectedPosNode, px, py, pz)
	setRotation(self.projectedPosNode, 0, yRot, 0)
	if courseplay.debugChannels[12] then
		drawDebugLine(px, py + 3, pz, 1, 1, 0, px, py + 1, pz, 1, 1, 0);
		DebugUtil.drawDebugNode(self.relevantWpNode, string.format('ix = %d\nd = %.1f\nrelevant\nnode', self.relevantIx, dz))
		DebugUtil.drawDebugNode(self.projectedPosNode, 'projected\nvehicle\nposition')
	end
end

-- Now, from the relevant section forward we search for the goal point, which is the one
-- lying lookAheadDistance in front of us on the path
function PurePursuitController:findGoalPoint()
	local epsilon = 0.01
	local d
	local tx, ty, tz

	local vx, vy, vz = getWorldTranslation(self.vehicle.cp.DirectionNode or self.vehicle.rootNode);

	local cx, cy, cz = getWorldTranslation(self.relevantWpNode)
	local dFromCurrent = courseplay:distance(cx, cz, vx, vz)
	local nx, ny, nz = getWorldTranslation(self.nextWpNode)
	local dFromNext = courseplay:distance(nx, nz, vx, vz)

	if dFromCurrent > self.lookAheadDistance and dFromNext > self.lookAheadDistance then
		-- too far from our next waypoint, set the goal to the current WP
		self:setNodeToWaypoint(self.goalNode, self.relevantIx)
		DebugUtil.drawDebugNode(self.goalNode, string.format('relevantix = %d\ntoo\nfar', self.relevantIx))
		return
	end

	-- create a node at current wp node. We'll move this up on the path until we reach the segment
	-- in lookAheadDistance
	local helperNode = courseplay.createNode( self.name .. 'helperNode', 0, 0, 0)
	setTranslation(helperNode, cx, cy, cz)
	local _, yRot, _ = getRotation(self.relevantWpNode)
	setRotation(helperNode, 0, yRot, 0)

	local isGoalPointValid = false

	local ix = self.relevantIx + 1
	while ix <= #self.vehicle.Waypoints do
		nx, ny, nz = self:getWaypointPosition(ix)
		-- di	stance between the vehicle position and the next waypoint
		d = courseplay:distance(vx, vz, nx, nz)
		if d > self.lookAheadDistance then
			-- our goal point is now between ix and ix + 1, let's find it
			-- distance between current and next waypoint
			local dToNext = courseplay:distance(cx, cz, nx, nz)
			local minDz, maxDz, currentDz, currentRange = 0, dToNext, dToNext / 2, dToNext
			-- this is the current waypoint for the rest of Courseplay, the waypoint we are driving to
			self:setNodeToWaypoint(self.currentWpNode, ix)
			self.currentIx = ix

			while currentRange > epsilon do
				local gx, gy, gz = localToWorld(helperNode, 0, 0, currentDz)
				d = courseplay:distance(vx, vz, gx, gz)
				if d < self.lookAheadDistance + epsilon and d > self.lookAheadDistance - epsilon then
					-- we are close enough to the goal point.
					gy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, gx, 0, gz)
					setTranslation(self.goalNode, gx, gy, gz)
					isGoalPointValid = true
					tx, ty, tz = localToWorld(self.goalNode, 0, 0, 0)
					if courseplay.debugChannels[12] then
						DebugUtil.drawDebugNode(self.goalNode, string.format('ix = %d\nd = %.1f\ncr = %.1f\ngoal\npoint', ix, d, currentRange))
						DebugUtil.drawDebugNode(self.currentWpNode, string.format('ix = %d\ncurrent\nwaypoint', ix))
						drawDebugLine(gx, gy + 3, gz, 0, 1, 0, gx, gy + 1, gz, 0, 1, 0);
					end
					break
				end
				if d < self.lookAheadDistance then
					minDz = currentDz
				else
					maxDz = currentDz
				end
				currentRange = currentRange / 2
				currentDz = minDz + currentRange
			end
			break
		end
		-- move current position to the next waypoint
		setTranslation(helperNode, nx, ny, nz)
		setRotation(helperNode, 0, math.rad( self.vehicle.Waypoints[ix].angle), 0)
		--DebugUtil.drawDebugNode(helperNode, string.format('ix = %d\nhelper\nnode', ix))
		cx, cy, cz = nx, ny, nz
		ix = ix + 1
	end
	courseplay.destroyNode(helperNode)
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
	return courseplay.debugChannels[7]
end

function PurePursuitController:getCurrentWaypointPosition()
	local cx, cz
	if self:isActive() then
		cx, _, cz = getWorldTranslation(self.currentWpNode)
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

function PurePursuitController:shouldChangeWaypoint(distToChange)
	if self:isActive() then
		-- true when at the last waypoint to trigger the last waypoint processing in drive.lua (which was triggerd by
		-- the distToChange condition before PPC
		shouldChangeWaypoint = self:getCurrentWaypointIx() ~= self.vehicle.cp.waypointIndex or self:atLastWaypoint()
	else
		shouldChangeWaypoint = self.vehicle.cp.distanceToTarget <= distToChange
	end
end

function PurePursuitController:atLastWaypoint()
	if self:isActive() then
		-- check for relevantIx for safety, if we are beyond that, our course is done
		return self.currentIx == #self.vehicle.Waypoints or self.relevantIx == #self.vehicle.Waypoints
	else
		return not self.vehicle.cp.waypointIndex < self.vehicle.cp.numWaypoints
	end
end
