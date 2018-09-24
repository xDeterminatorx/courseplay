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

Waypoint = {}
Waypoint.__index = Waypoint


function Waypoint.switchingToReverseAt(vehicle, ix)
	return (not vehicle.Waypoints[ix].rev) and vehicle.Waypoints[math.min(ix + 1, #vehicle.Waypoints)].rev 		
end

function Waypoint.switchingToForwardAt(vehicle, ix)
	return (vehicle.Waypoints[ix].rev) and not vehicle.Waypoints[math.min(ix + 1, #vehicle.Waypoints)].rev		
end

-- constructor from the legacy Courseplay waypoint
function Waypoint:new(cpWp, cpIndex)
	local newWp = {}
	setmetatable( newWp, self )
	newWp:set(cpWp, cpIndex)
	return newWp
end

function Waypoint:set(cpWp, cpIndex)
	-- we initialize explicitly, no table copy as we want to have
	-- full control over what is used in this object
	self.x = cpWp.cx or 0
	self.z = cpWp.cz or 0
	self.angle = cpWp.angle or 0
	self.rev = cpWp.rev or false
	self.cpIndex = cpIndex or 0
end

function Waypoint:getPosition()
	local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.x, 0, self.z)
	return self.x, y, self.z
end

-- a node related to a waypoint
WaypointNode = {}
WaypointNode.MODE_NORMAL = 1
WaypointNode.MODE_LAST_WP = 2
WaypointNode.MODE_SWITCH_DIRECTION = 3
WaypointNode.MODE_SWITCH_TO_FORWARD = 4

WaypointNode.__index = WaypointNode

function WaypointNode:new(name, vehicle, logChanges)
	local newWaypointNode = {}
	setmetatable( newWaypointNode, self )
	newWaypointNode.vehicle = vehicle
	newWaypointNode.logChanges = logChanges
	newWaypointNode.node = courseplay.createNode(name, 0, 0, 0)
	return newWaypointNode
end

function WaypointNode:destroy()
	courseplay.destroyNode(self.node)
end

function WaypointNode:getWaypointPosition(ix)
	local x, z = self.vehicle.Waypoints[ix].cx, self.vehicle.Waypoints[ix].cz
	local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
	return x, y, z
end

function WaypointNode:setToWaypoint(ix)
	if ix ~= self.ix and self.logChanges then
		courseplay.debugVehicle(12, self.vehicle, 'PPC: %s waypoint index %d', getName(self.node), ix)
	end
	self.ix = math.min(ix, #self.vehicle.Waypoints)
	local x, y, z = self:getWaypointPosition(self.ix)
	setTranslation(self.node, x, y, z)
	setRotation(self.node, 0, math.rad(self.vehicle.Waypoints[self.ix].angle), 0)
end


-- Allow ix > #Waypoints, in that case move the node lookAheadDistance beyond the last WP
function WaypointNode:setToWaypointOrBeyond(ix, distance)
	if ix > #self.vehicle.Waypoints then
		-- beyond the last, so put it on the last for now
		-- but use the direction of the one before the last as the last one's is bogus
		self:setToWaypoint(#self.vehicle.Waypoints - 1)
		local _, yRot, _ = getRotation(self.node)
		self:setToWaypoint(#self.vehicle.Waypoints)
		setRotation(self.node, 0, yRot, 0)
		-- And now, move ahead a bit.
		local nx, ny, nz = localToWorld(self.node, 0, 0, distance)
		setTranslation(self.node, nx, ny, nz)
		if self.mode and self.mode ~= WaypointNode.MODE_LAST_WP then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: last waypoint reached, moving node beyond last: %s', getName(self.node))
		end
		self.mode = WaypointNode.MODE_LAST_WP
	elseif Waypoint.switchingToReverseAt(self.vehicle, ix) or Waypoint.switchingToForwardAt(self.vehicle, ix) then
		-- just like at the last waypoint, if there's a direction switch, we want to drive up
		-- to the waypoint so we move the goal point beyond it
		-- the angle of ix is already pointing to reverse here
		self:setToWaypoint(ix)
		-- turn node back as this is the one before the first reverse, already pointing to the reverse direction.
		local _, yRot, _ = getRotation(self.node)
		setRotation(self.node, 0, yRot + math.pi, 0)
		-- And now, move ahead a bit.
		local nx, ny, nz = localToWorld(self.node, 0, 0, distance)
		setTranslation(self.node, nx, ny, nz)
		if self.mode and self.mode ~= WaypointNode.MODE_SWITCH_DIRECTION then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: switching direction at %d, moving node beyond it: %s', ix, getName(self.node))
		end
		self.mode = WaypointNode.MODE_SWITCH_DIRECTION
	else
		if self.mode and self.mode ~= WaypointNode.MODE_NORMAL then
			courseplay.debugVehicle(12, self.vehicle, 'PPC: normal waypoint (not last, no direction change: %s', getName(self.node))
		end
		self.mode = WaypointNode.MODE_NORMAL
		self:setToWaypoint(ix)
	end
end

