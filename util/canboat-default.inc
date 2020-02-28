<?php
#
# (C) 2009-2020, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
#
# CANboat is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CANboat is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with CANboat.  If not, see <http://www.gnu.org/licenses/>.
#

  function getCanboatDefault($variable, $default)
  {
    $fd = @fopen('/etc/default/canboat', 'r');
    if ($fd and is_resource($fd))
    {
      while (!feof($fd))
      {
	$s = fgets($fd, 8192);
        $matches = array();
        if (preg_match('/^ *([A-Za-z0-9_]+) *= *(.+)/', $s, $matches))
        {
          if (strtolower($matches[1]) == strtolower($variable))
          {
	    return eval('return '.$matches[2].';');
          }
        }
      }
      fclose($fd);
    }
    return $default;
  } 


?>

