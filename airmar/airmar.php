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

if ($cmd == "request-access-level")
{
  request_access_level();
}
else
{
  usage();
}

function usage()
{
  echo "Usage: airmar.php <dest> <command>\n\n";
  echo "where command is one of:\n";
  echo "    request-access-level\n";
  echo "and <dest> is the decimal device number of the Airmar sensor on the CAN bus, or 255 for broadcast\n";
  exit(1);
}

function request_access_level()
{
  global $dest;

  # Usage: ./rel/linux-i586/send-group-function <dest> <prio> <pgn> <field>=<value> ...
  $command = shell_exec("/usr/local/bin/send-group-function $dest 5 65287 1=135 3=4");
  echo $command;

  $response = n2k_request_response($command, 'Request Access Level');
  if (is_array($response[126208][$dest."_0"]))
  {
    echo "Looks like the sensor denied the request.\n";
  }
  if (is_array($response[65287]))
  {
    echo "Some response received\n";
    $response = $response['65287'];
    print_r($response);
  }
}

function n2k_request_response($request, $description = 'N2K')
{
  $errno = 0;
  $errstr = '';
  $n2k = @fsockopen('localhost', 2597, &$errno, &$errstr, 15);
  if (!$n2k)
  {
    echo "Cannot connect to N2KD: $errstr\n";
    exit(1);
  }

  #
  # Ask for device list
  #

  fwrite($n2k, $request."\n");
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
?>
