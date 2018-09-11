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

-- constructor from the legacy Courseplay waypoint
function Waypoint:new(cpWp, cpIndex)
	local newWp = {}
	setmetatable( newWp, self )
	-- we initialize explicitly, no table copy as we want to have
	-- full control over what is used in this object
	newWp.x = cpWp.cx
	newWp.z = cpWp.cz
	newWp.angle = cpWp.angle
	newWp.rev = cpWp.rev
	newWp.cpIndex = cpIndex
	return newWp
end

function Waypoint:getPosition()
	local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, self.x, 0, self.z)
	return self.x, y, self.z
end

Node = {}
Node.__index = Node

function Node:new(name, x, z, yRotation, rootNode ) 
	local newNode = {}
	newNode.node = courseplay.createNode(name, x, z, yRotation, rootNode)
	return newNode
end

WaypointNode = {}
WaypointNode.__index = WaypointNode

function WaypointNode(name, cpWp, cpIndex) 
	local newWaypointNode = {}
	newWaypointNode.waypoint = Waypoint:new(cpWp, cpIndex)
	newWaypointNode.node = Node:new(name, newWaypointNode.waypoint.x, newWaypointNode.waypoint.z)
	return newWaypointNode
	
end