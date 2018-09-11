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