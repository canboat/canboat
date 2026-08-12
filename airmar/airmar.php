#!/usr/bin/php5
<?php
if (!is_array($argv))
{
  echo "airmar.php is only for command-line usage right now.\n";
  return;
}
if ($argc < 3)
{
  usage();
}

$dest = $argv[1];
$cmd  = $argv[2];

if ($cmd == "set-depth-offset" && $argc == 4)
{
  set_depth_offset($argv[3]);
}
else if ($cmd == "request-access-level")
{
  request_access_level();
}
else if ($cmd == "set-access-level" && $argc == 4)
{
  set_access_level($argv[3]);
}
else if ($cmd == "request-speed-calibration")
{
  request_speed_calibration();
}
else
{
  usage();
}

function usage()
{
  echo "Usage: airmar.php <dest> <command>\n\n";
  echo "where command is one of:\n";
  echo "    set-depth-offset <offset> in millimeters\n";
  echo "    request-access-level\n";
  echo "    set-access-level <level> (level 0 and 1 supported)\n";
  echo "    reuqest-speed-calibration\n";
  echo "and <dest> is the decimal device number of the Airmar sensor on the CAN bus, or 255 for broadcast\n";
  exit(1);
}

function set_depth_offset($offset)
{
  global $dest;

  $command = shell_exec("command-group-function $dest 5 128267 3=" . sprintf("%04x", $offset));
  $analyzed = shell_exec("echo '$command' | analyzer 2>/dev/null");
  echo "Sending request to device $dest: $analyzed\n";

  $response = n2k_request_response($command, 'set-depth-offset');
  show_command_response($response);

  Sleep(0.2);
  $response = n2k_request_response(null, 'set-depth-offset');
  if (array_key_exists(128267, $response) && is_array($response[128267]))
  {
    echo "New depth sentence\n";
    print_r($response[128267]);
  }
}

function request_access_level()
{
  global $dest;

  $command = shell_exec("request-group-function $dest 5 65287 1=0087 3=04");
  $analyzed = shell_exec("echo '$command' | analyzer 2>/dev/null");
  echo "Sending request to device $dest to report PGN 65287: $analyzed\n";

  $response = n2k_request_response($command, 'Request Access Level');

  show_command_response($response);
  Sleep(0.2);
  $response = n2k_request_response(null, 'request-access-level');
  show_airmar_data($response);
}

function set_access_level($level)
{
  global $dest;

  $command = shell_exec("command-group-function $dest 5 65287 1=0087 3=04 4=01 5=".sprintf("%02x", $level)." 7=12345678");
  $analyzed = shell_exec("echo '$command' | analyzer 2>/dev/null");
  echo "Sending request to device $dest to report PGN 65287: $analyzed\n";

  $response = n2k_request_response($command, 'Set Access Level');

  $command = shell_exec("request-group-function $dest 5 65287 1=0087 3=04");

  show_command_response($response);
  Sleep(2);
  $response = n2k_request_response(null, 'request-access-level');
  show_airmar_data($response);
}

function request_speed_calibration()
{
  global $dest;

  $command = shell_exec("request-group-function $dest 5 126208 1=0087 3=04 4=29");
  echo "Sending request to device $dest to report PGN 126208 PID 41: $command\n";
  $analyzed = shell_exec("echo '$command' | analyzer 2>/dev/null");
  echo "Sending request to device $dest to report PGN 126208 PID 41: $analyzed\n";

  $response = n2k_request_response($command, 'Request Speed Calibration');

  show_command_response($response);
  Sleep(2);
  $response = n2k_request_response(null, 'request-access-level');
  show_airmar_data($response);
}

function n2k_request_response($request, $description = 'N2K')
{
  $errno = 0;
  $errstr = '';
  $n2k = @fsockopen('localhost', 2597, $errno, $errstr, 15);
  if (!$n2k)
  {
    echo "Cannot connect to N2KD: $errstr\n";
    exit(1);
  }

  #
  # Ask for device list
  #

  if ($request !== null)
  {
    fwrite($n2k, $request."\n");
  }
  $s = '';
  while (!feof($n2k))
  {
    $s .= fgets($n2k, 1024);
  }
  fclose($n2k);
  $data = json_decode($s, true);
  if (!is_array($data))
  {
    echo "Error: received invalid response for $description request\n";
    exit(1);
  }
  return $data;
}

function show_command_response($response)
{
  global $dest;

  if (array_key_exists(126208, $response) && is_array($response[126208]))
  {
    if (array_key_exists($dest, $response[126208]) && is_array($response[126208][$dest]))
    {
      $fields = $response[126208][$dest]['fields'];
      echo "Device response: ". $fields['Function Code'] . " for PGN " . $fields['PGN'];
      if ($fields['Parameter Error'] != 0)
      {
        echo " - ERROR\n";
      }
      else
      {
        echo " - OK\n";
      }
    }
  }
}

function show_airmar_data($response)
{
  global $dest;

  foreach ($response as $key => $data)
  {
    foreach ($data as $device => $answer)
    {
      if ($device == $dest && array_key_exists('Manufacturer Code', $answer['fields']) && $answer['fields']['Manufacturer Code'] == 'Airmar')
      {
        print_r($data);
        break;
      }
    }
  }
}
?>
