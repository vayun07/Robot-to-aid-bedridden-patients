<?php

	// Load JSON state
    $string = file_get_contents("robotStateNew.json");
    $json_a= json_decode($string,true);
	
	//first part is the name of the value in the json text file that will be read by server.php when called from Arduino
	//second part is the URL value that is parsed.
    $json_a['mode'] = $_GET["URLmode"];
    $json_a['xval'] = $_GET["URLxval"];
    $json_a['yval'] = $_GET["URLyval"];
	$json_a['udlr'] = $_GET["URLudlr"];
	$json_a['cmdVal'] = $_GET["URLcmdVal"];

    $fp = fopen('robotStateNew.json', 'w');
    fwrite($fp, json_encode($json_a));
    fclose($fp);

	// Create a TCP/IP socket & connect to the server
	$socket = socket_create(AF_INET, SOCK_STREAM, SOL_TCP);
	$result = socket_connect($socket, "192.168.100.5", "80");

	// Request
	$in = "HEAD / HTTP/1.1\r\n";
	$in .= "Content-Type: text/html\r\n";
	$in .= $json_a['mode'] . "," . 
	$json_a['xval'] . "," .
	$json_a['yval'] . "," .
	$json_a['udlr'] . "," .
	$json_a['cmdVal'] . ",\r\n\r\n";
	$out = '';

	// Send request
	socket_write($socket, $in, strlen($in));
	
	// Read answer
	while ($out = socket_read($socket, 4096)) {
	  echo $out;
	}

	// Close socket
	socket_close($socket);
	
?>


