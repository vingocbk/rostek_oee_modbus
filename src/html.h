#ifndef html_H
#define html_H


char index_html_handle_root[]  = 
	"<!DOCTYPE HTML><html><head>\n"
		"<title>ESP Input Form</title>\n"
		"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
		"</head><body>\n"
            "<h1>CONFIG MODBUS</h1><br>\n"
			"<form action=\"/config_modbus\">\n"
				"<h3>Slave ID</h3><br>\n"
				"<input type=\"text\" name=\"slave_id\">\n"
				"%d\n"
				// "<input type=\"submit\" value=\"Save\"><br>\n"
				"<h3>Baudrate</h3><br>\n"
				"<input type=\"text\" name=\"baudrate\">\n"
				"%d<br>\n"
				"<input type=\"submit\" value=\"Save Config\"><br>\n"
			"</form><br>\n"
			"<h1>RESET DATA</h1><br>\n"
			"<form action=\"/reset_data\">\n"
				"<input type=\"submit\" value=\"Reset Data\"><br>\n"
			"</form><br>\n"
	    "</body>\n"
    "</html>";


//TEMPERATURE
const char index_html_handle_done_setup[] PROGMEM = R"rawliteral(
	<!DOCTYPE HTML><html><head>
		<title>ESP Input Form</title>
		<meta name="viewport" content="width=device-width, initial-scale=1">
		</head><body>
		<center>
			<h1>DONE SETUP TIME</h1><br>
			<br><a href=\><button>Back to Main</button></a><br>
		</center>
	</body></html>)rawliteral";


#endif