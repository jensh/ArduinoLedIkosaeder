#!/usr/bin/env node
//
// (c) 2015-01-10 Jens Hauke <jens.hauke@4k2.de>
// (c) 2015-03-30 Jens Hauke <jens.hauke@4k2.de>
//

var serialPort = require("serialport");

// Docu:  https://www.npmjs.com/package/serialport

//var serialDevicePath = "/dev/rfcomm2";
var serialDevicePath = "/dev/ttyUSB0";


function Parser() {
	this.cmd_number = 0;
	this.cmd_number_neg = false;
	this.values = {};
}

Parser.prototype.push = function (ch) {
	var cmd = String.fromCharCode(ch);
	switch (cmd) {
	case '-':
		this.cmd_number_neg = true;
		var number_cmd = true;
		break;
	case '0': case '1': case '2': case '3': case '4':
	case '5': case '6': case '7': case '8': case '9':
		if (this.cmd_number === undefined) this.cmd_number = 0;
		this.cmd_number = this.cmd_number * 10;
		this.cmd_number += this.cmd_number_neg ? -cmd : +cmd;
		var number_cmd = true;
		break;
	case '\r':
	case '\n':
		// console.log('\n');
		break;
	default:
		if (ch >= 0 && ch < 128) {
			if (this.cmd_number !== undefined) {
				this.values[cmd] = this.cmd_number;
				if (this['set_' + cmd]) this['set_' + cmd](this.cmd_number);
			} else {
				if (this['get_' + cmd]) this['get_' + cmd]();
			}
		}
	}
	if (!number_cmd) {
		this.cmd_number = undefined;
		this.cmd_number_neg = false;
	}
}

parser = new Parser();

function Ikosaeder(sconn, error_check) {
	this.sconn = sconn;
	this.error_check = error_check;
}

Ikosaeder.prototype.cmd_s = function (led_num) {
	 // Switch on led_num
	this.sconn.write("" + (led_num|0) + "s", this.error_check);
}

Ikosaeder.prototype.cmd_t = function (lo_hi, in_out) {
	// Set (D7 D6 D5 D4 D3 D2 D9 D8) to lo, hi or tri-state
	this.sconn.write("" + (in_out|0) + "," + (lo_hi|0) + "t", this.error_check);
}

Ikosaeder.prototype.cmd_f = function (pin_lo, pin_hi) {
	// route from pin_lo (-) to pin_hi (+)
	var lo_hi = 1 << pin_hi;
	var in_out = lo_hi | 1 << pin_lo;

	this.cmd_t(lo_hi, in_out);
}


function communicate(callback) {
	/* Open */
	var interval;
	var sconn = new serialPort.SerialPort(serialDevicePath, {
		// Baud rate:
		// 115200, 57600, 38400, 19200, 9600 (default),
		// 4800, 2400, 1800, 1200, 600, 300, 200, 150, 134, 110, 75, 50.
		// Custom rates as allowed by hardware.
		baudrate: 115200, // 9600,
		databits: 8, // 8(default), 7, 6, or 5.
		stopbits: 1, // 1(default), 2
		parity: 'none' // 'none'(default), 'even', 'mark', 'odd', 'space'
	});


	sconn.on("open", function () {
		console.log('open');

		sconn.on('data', function(data) {
			for (var i = 0; i < data.length; i++) {
				parser.push(data[i]);
			}
		});

		var ikosaeder = new Ikosaeder(sconn, error_check);

		interval = setInterval(function () {
			step(ikosaeder);
		}, INTERVAL);

		process.on("SIGINT", function () {
			console.log("Bye-bye");
			sconn.write("ss"); // Switch off
			done();
		});
	});

	sconn.on("error", function (err) {
		console.log('Serialport: Error: ' + (err ? err : 'ok'));
		done(err);
	});

	sconn.on("close", function (err) {
		console.log('Serialport: Closed: ' + (err ? err : 'ok'));
		done(err);
	});

	function error_check(err) {
		if (err) done(err);
	}

	function done(err) {
		if (interval) clearInterval(interval);
		interval = null;

		sconn.close(function (err) {});

		if (callback) callback(err);
		callback = null;
	}
}

var intervals = [];

communicate(function (err) {
	console.log('Communication done');
	intervals.forEach(function (interval) {
		clearInterval(interval);
	});
});


parser.set_Z = function () {
	// console.log(JSON.stringify(parser.values, null, '\t'));
};

var INTERVAL = 30;
var num = 0;
var sequence = [
	//	[0, 1], [1, 6], [1, 2], [2, 6], [2, 0], [0, 6] // level 0
	//	[1, 0], [3, 0], [1, 3],     [2, 1], [4, 1], [2, 4],    [0, 2], [5, 2], [0, 5], // level 1
	// [4, 0], [4, 2], [2, 5]

	[0, 6], // level 0
	[0, 1], [0, 1], [2, 0], [2, 0], [0, 5], [0, 5], [1, 0], [1, 0], [3, 0], [3, 0], // level 1
	[1, 6], [1, 2], [2, 6], [5, 2], [6, 5], [5, 1], [6, 1], [3, 1], [6, 3], [1, 3], // level 2
	[2, 1], [4, 1], [2, 4], [0, 2], [5, 0], [4, 5], [1, 4], [1, 5], [5, 3], [3, 2], // level 3
	[6, 2], [4, 2], [6, 4], [4, 0], [6, 0], [0, 4], [4, 6], [5, 4], [5, 6], [2, 5], // level 4
	[2, 3], [2, 3], [3, 4], [3, 4], [0, 3], [0, 3], [4, 3], [4, 3], [3, 5], [3, 5], // level 5
	[3, 6],  // level 6
	[2, 3], [2, 3], [3, 4], [3, 4], [0, 3], [0, 3], [4, 3], [4, 3], [3, 5], [3, 5], // level 5
	[6, 2], [4, 2], [6, 4], [4, 0], [6, 0], [0, 4], [4, 6], [5, 4], [5, 6], [2, 5], // level 4
	[2, 1], [4, 1], [2, 4], [0, 2], [5, 0], [4, 5], [1, 4], [1, 5], [5, 3], [3, 2], // level 3
	[1, 6], [1, 2], [2, 6], [5, 2], [6, 5], [5, 1], [6, 1], [3, 1], [6, 3], [1, 3], // level 2
]

function step(ikosaeder) {
	// ikosaeder.cmd_s(num);
	ikosaeder.cmd_f(sequence[num][0], sequence[num][1]);
	console.log('step', num);

	num++;
	if (num >= sequence.length) num = 0;

}

// Local Variables:
//  compile-command: "./controller.js"
// End:
