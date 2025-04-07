<?php
$binaryStream = '';

while ($inputLine = trim(fgets(STDIN))) {
    if ($inputLine === '') {
        continue;
    }

    // Split the line into hex values using space as separator
    $hexValues = explode(' ', $inputLine);

    foreach ($hexValues as $hexValue) {
        // Convert each hex string to raw bytes
        echo hex2bin($hexValue);
    }
}
?>
