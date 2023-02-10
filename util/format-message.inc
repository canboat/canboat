<?php
#
# Format a particular N2K command
#
# Example: this will request all product information
#
# 1970-01-01-00:00:00.000,6,59904,0,255,3,14,f0,01
#
#
# (C) 2009-2023, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

  $format_n2k_messages = null;

  class FormatMessage
  {
    protected $variables;
    protected $data;
    protected $arguments;
    protected $format = "CANBOAT";

    public function __construct($message)
    {
      global $format_n2k_messages;

      if (!is_array($format_n2k_messages))
      {
        $format_n2k_messages = $this->getConfig();
      }
      $this->data = $format_n2k_messages[$message];
      $this->arguments = preg_split('/,/', $this->data['args']);
      if (!is_array($this->arguments))
      {
        die("Invalid message '$message' is not know by n2k-message-db.json");
      }
    }

    public function setFormat($format)
    {
      if ($format == "YDWG")
      {
        $this->format = "YDWG";
      } elseif ($format == "CANBOAT")
      {
        $this->format = "CANBOAT";
      }
      else
      {
        die("Invalid format $format for setFormat()");
      }
    }


    function getArgs()
    {
      return $this->arguments;
    }

    function eval_expr($expr)
    {
      if (is_array($this->variables) && count($this->variables) )
      {
        foreach($this->variables as $var_name => $var_value)
        {
          $$var_name = $var_value;
        }
      }
      $return_value = '';
      eval('$return_value = ' . $expr . ';');
      return $return_value;
    }

    function getConfig()
    {
      global $debug;

      $s = @file_get_contents('n2k-message-db.json', true);
      if (!$s)
      {
        echo "unable to read config file n2k-message-db.json\n";
        exit(1);
      }
      $data = json_decode($s, true);
      if (!$data)
      {
        echo "unable to parse config file n2k-message-db.json\n";
        exit(1);
      }
      return $data;
    }

    function numeric_value($str)
    {
      if (substr($str, 0, 2) === '0x')
      {
        return hexdec(substr($str, 2));
      }
      return $str;
    }

    public function setArg($name, $value)
    {
      $this->variables[$name] = $this->numeric_value($value);
    }

    public function formatMessage($dst)
    {
      $pgn  = $this->data['pgn'];
      $expr = $this->data['data'];

      #
      # Now evaluate '$expr'
      #
      $data = $this->eval_expr($expr);

      # 1970-01-01-00:00:00.000,6,59904,0,255,3,14,f0,01
      $prio = 6;
      $src = 0; /* Unimportant, NGT will use its own address */
      $len = strlen($data);
      $data = unpack('H*', $data);
      $data = $data[1];

      if ($this->format == "CANBOAT")
      {
        $timestamp = strftime('%F-%T.000');
        $data = preg_replace('/(..)/', ',$1', $data);
        return $timestamp.",$prio,$pgn,$src,$dst,$len".$data."\n";
      }
      // else YDWG
      if (($pgn & 0xff) == 0)
      {
        $pgn = $pgn + $dst;
      }
      $msgid = ($prio << 26) | ($pgn << 8) | ($src & 0xff);

      $data = preg_replace('/(..)/', ' $1', $data);
      printf("%08X%s\r\n", $msgid, $data);
    }

  }

?>
