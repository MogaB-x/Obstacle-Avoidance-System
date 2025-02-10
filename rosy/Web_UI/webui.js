//Network connection parameters
local_network = true 
local_IP = "192.168.53.159"
robot_ws_port = 9090

robot_URL = "ws://" + local_IP + ":" + robot_ws_port

class ROSBridge extends ROSLIB.Ros {
    constructor(URL, do_auth=false) {
        super({url: URL});
				this.on("connection", this.after_connection.bind(this));
				 		
				this.on("error", function (error) {
						console.log("Error connecting to websocket server: ", error);
				});
				
				this.on("close", function () {
						console.log("Connection to websocket server closed.");
				});
				if(do_auth) this.do_auth();
    }
	after_connection() {
		console.log("Connected to websocket server.");
	}
	do_auth() {
		let secret = 'rdfgdg1';
		let dest = "rosy";
		let rand = Math.random().toString(16);
		let time = Math.round(new Date().getTime() / 1000);
		let timeEnd = time + 10000;
		let level = "admin";
		let client = "t9";
		this.authenticate(
			SHA512(secret + 
				client + dest + rand + parseInt(time).toString() + level + parseInt(timeEnd).toString())
		, client, dest, rand, time, level, timeEnd);
		console.log("Auth to websocket server.");
	}
}

class ROSServ extends ROSLIB.Service {
	constructor(ros, name, r, stype="std_srvs/Empty") {
		super({ros: ros, name: r + name, serviceType: stype});
	}
	call(v) {
		console.log("starting service");
		var request = new ROSLIB.ServiceRequest((undefined==v)?{}:{data : v});
		this.callService(request, function(result) {
				console.log("result", result.success, result.message)
		});
	}
}

//Clasa de baza pentru abonare si receptionarea mesajelor pe topic
class ROSTSB extends ROSLIB.Topic {
	constructor(o, callback, origin_obj) {
		super(o);
		super.subscribe(this.on_msg.bind(this));
		this.last_msg = null;
		this.last_ts = null;
		this.callback = callback;
		this.origin = origin_obj;
		this.value = null;
	}
	//Stocheaza ultimul mesaj primit si timestamp-ul asociat
	//Apeleaza o functie de callback cand primeste un mesaj nou
	on_msg(m) {
		this.last_msg = m;
		this.last_ts = new Date();
		if(this.callback) {
			if(this.origin) this.callback.bind(this.origin)(m);
			else this.callback(m);
		}
	}
}

//topic ROS tip subscriber in format JSON fara ignorare pachete care depasesc latimea de banda disponibila
class ROSTS extends ROSTSB {
	constructor(ros, topic, msgt, callback, origin_obj, root, inter_msg_min_time_ms=0) {
		super({ ros: ros, name: root + topic, messageType: msgt }, callback, origin_obj);
	}
}

class ROSTSL extends ROSTSB {
	constructor(ros, topic, msgt, callback, origin_obj, root, inter_msg_min_time_ms=0) {
		super({ ros: ros, name: root + topic, messageType: msgt, 
			queue_length: 1, throttle_rate: inter_msg_min_time_ms 
		}, callback, origin_obj);
	}
}

//publisher
class ROSTP extends ROSLIB.Topic {
	constructor(ros, topic, msgt, msgi, interval=0, root) {
		super({ ros: ros, name: root + topic, messageType: msgt });
		this.advertise.bind(this);
		this.msg = new ROSLIB.Message(msgi);
		if(interval) {
				this.interval = setInterval(this.publish.bind(this), interval);
		} else {
				this.interval = null;
		}
	}
	publish(msg=null) {
		if(msg != null) this.msg = msg;
		if(this.ros.isConnected) super.publish(this.msg);
	}
}

class Point {
	//coordonate si unitate de masura
	constructor(x = 0, y = 0, z = 0, unit = "cm") {
		this.unit = unit;
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

class Path {
	//culoarea traseului, delta minima intre puncte, numar limita puncte si raza desenare punctelor
	constructor(col, delta = 20, lim = 4*1024, pointr=1) {
		this.col = col;
		this.path = [];
		this.lim = lim;
		this.point_radius = pointr;
		this.path_min_delta = delta;
	}
	//adauga un nou punct pe traseu
	add(x, y) {
		let ul = this.path.length - 1;
		//verifica daca noul punct respecta delta minim fata de ultimul punct
		if (
			ul < 0 ||
			Math.abs(this.path[ul][0] - x) >= this.path_min_delta ||
			Math.abs(this.path[ul][1] - y) >= this.path_min_delta
		) {
			this.path.push([x, y]);
			while (this.path.length > this.lim) this.path.shift();
		}
	}
	draw(c) {
		for (let p in this.path)
			drawPoint(c, this.path[p][0] / 10, this.path[p][1] / 10, this.point_radius, this.col);
	}
	clear() {
		this.path = [];
	}
}

class LiDAR {
	constructor(ros, r = "/r0/", col = "black", l = "/scan") {
		this.scan = null;
		this.col = col;
		this.drawn = true;
		this.lidarsubscriber = new ROSTSL(ros,l,"sensor_msgs/LaserScan",this.on_scan,this,r);
	}
	on_scan(m) {
		this.scan = m; 
	}
	draw(c, points = 1) {
		if (null != this.scan) {
			this.drawn = true;
			//cercle radius
			let cr = 5;
			//drawCircle(c, 0, 0, cr, this.col);
			c.beginPath();
			c.arc(0, 0, cr, this.scan.angle_min, this.scan.angle_max);
			c.strokeStyle = this.col;
			c.lineWidth = 1;
			c.stroke();
			drawLine(c, 0, 0, cr*Math.cos(this.scan.angle_min), cr*Math.sin(this.scan.angle_min));
			drawLine(c, 0, 0, cr*Math.cos(this.scan.angle_max), cr*Math.sin(this.scan.angle_max));
			console.log(" scan.angle_min: "+ this.scan.angle_min + " angle_max " + this.scan.angle_max + " angle increment: "+ this.scan.angle_increment)

			//console.log('lidar ' + this.scan.angle_min*180/Math.PI+" "+ this.scan.angle_max*180/Math.PI) 
			let r = this.scan.ranges;
			let ts = this.scan.angle_min;
			let td = this.scan.angle_increment; //2*Math.PI/r.length;
			for (i = 0; i < r.length; i++) {
				let rr = r[i] * 100;
				let a = ts + i * td;
				let x = rr * Math.cos(a);
				let y = rr * Math.sin(a);
				c.fillStyle = this.col;
				let ps = points;
				c.fillRect(x - (ps >> 1), y - (ps >> 1), ps, ps);
			}
		}
	}
}

class Rosy extends Point {
    constructor(ros, r='/r0/') {
        super(0, 0, 0);
		this.rosd=ros;
		this.ns = r;
		
		this.RW = 62;
		this.RL = 55;
		
		this.script = [[1, 0, 10], [1, 1, 10], [0, 1, 10], [0, 0, 10]];
		document.getElementById('cmb_mode').value = "0";
		this.script_scale = 1.0;
		this.script_start = null;
		this.script_step = -1;
		this.script_state = 0;
		this.mode_prev = 0;
		
        this.path_odo = new Path("orange", 10); 
		this.path_ref = new Path("red", 10, 100, 2);
		
        this.pose = new ROSTSL(ros, "pose", "geometry_msgs/Point", this.on_pose, this, r, 20);
        this.battery_voltage = new ROSTSL(ros, "battery/voltage", "std_msgs/Float32", this.on_battery_voltage, this,  r, 1000);
		this.lidar_front = new LiDAR(ros, "black", "scan");		
        this.cmdVel = new ROSTP(ros, "cmd_vel", "geometry_msgs/Point", {x:0, y:0, z:0}, 0, r);
		this.cmdPose = new ROSTP(ros, "cmd_pose", "geometry_msgs/Point", {x:0, y:0, z:0}, 0,  r);
		this.cmdMode = new ROSTP(ros, "cmd_mode", "std_msgs/Int8", { data: 0 } , 0, r);
	
    }

	setODOPose() {
		document.getElementById('setx').value=Math.round(rosy.x);
		document.getElementById('sety').value=Math.round(rosy.y);
		document.getElementById('setz').value=Math.round(rosy.Z*10)/10;
	}

	change_mode(value) {
		this.cmdMode.msg.data = parseInt(value);
		console.log(value)
		//mod dezactivat
		if (value === 0) {
			this.cmdMode.msg.data = 0x00;
		//mod GTP Rotation-Translation
		}else if (value === 1) {
			this.cmdMode.msg.data = 0x01;
		//mod GTP Translation-Rotation
		}else if (value === 2) {
			this.cmdMode.msg.data = 0x02;
		//mod GTP All
		}else if (value === 3) {
			this.cmdMode.msg.data = 0x03;
		//mod Rotatie la stanga
		}else if (value === 4) {
			this.cmdMode.msg.data = 0x04;
		}

		this.cmdMode.publish();
	}

	sendG1(d) {
		this.cmdPose.msg.x = d[0];this.cmdPose.msg.y = d[1];this.cmdPose.msg.z = d[2];
		this.cmdPose.publish();
		console.log('G1', d);
	}

	cancelAction() {
		this.script_step = -1; //cancel script
		var mode_prev = this.cmdMode.msg.data;
		this.cmdMode.msg.data = 0;
		document.getElementById('cmb_mode').value = "0"; // actualizează valoarea dropdown-ului
		this.cmdMode.publish();
	}

    on_battery_voltage(m) {
        this.battery_voltage.value = Math.round(m.data * 10) / 10;        
    }

    on_pose(m) {
        if(true) {
			this.x = Math.round(m.x*1000);
			this.y = Math.round(m.y*1000);
			this.z = m.z;
			this.Z = Math.round((m.z * 1800) / Math.PI) / 10;
			this.path_odo.add(this.x, this.y);
		}
		this.path_odo.add( Math.round(m.x*1000),  Math.round(m.y*1000));
    }
	
	moveAction(x, y, z) {
		if(this.cmdVel != undefined)
		if (x !== undefined && y !== undefined && z !== undefined) {
			this.cmdVel.msg.x = x;
			this.cmdVel.msg.y = y;
			this.cmdVel.msg.z = z;
		} else {
			this.cmdVel.msg.x = 0;
			this.cmdVel.msg.y = 0;
			this.cmdVel.msg.z = 0;
		}
	}

    draw(c) {
        //rosy as a red square 60x50px data.cz rotation
        if (null != this.pose.last_msg) {
            c.save();
            c.translate(this.x / 10, this.y / 10);
            c.rotate(this.z);            
			c.fillStyle = "rgba(200, 0, 0, 0.5)";

			c.fillRect(-this.RL / 2, -this.RW / 2, this.RL, this.RW);

            drawCartesian(c, this.RL / 2, this.RW / 2.5);
			var delta, u1, deltax, deltay;
			var delta = -1.0*Math.PI/180;					
			var	u1 = Math.PI + delta;
			var	deltax = 16;
			var	deltay = 0;

			if(this.lidar_front) {
					c.save();
					c.translate(deltax, deltay);
					c.rotate(u1);
					this.lidar_front.draw(c, points);
					c.restore();
			}

            c.restore();
            this.path_odo.draw(c);
			this.path_ref.draw(c);
        }
    }
}

var rosy_next = null;			 //when robot selection changes this value takes the new value that is to be used on next drawing iteration
var rosy_refresh = false;	 //flag to trigger direct resource subscription/unsubscription for current robot
var rosy = null;
var ros = null;
var map = null;
var graph = null;
var plc = null;
var scale = 1;  //map scaling factor
var grids = 50; //grid spacing (cm)
var points = 4; //lidar point drawing size (cm)
var dest_x = 0; //(cm)
var dest_y = 0; //(cm)
var mouse_buttons = 0;
var tx = 0;
var ty = 0;
var translateStart = null;
var lastMouseDownCoord = null;
var interMouseDownMs = 1000;
var mouseIsPressed = false;
var lastMouseDownTS = new Date();
var checkshowplanningnodescheck = false;
var currentMousePos = null;
var mouseMoved = false;

function drawGrid(c) {
	s = grids * scale;
	d = map.height / s / 2;
	ox = -tx + map.width / 2;
	fx =  ox - map.width / 2;
	Fx =  ox + map.width / 2;

	oy = -ty + map.height / 2;
	fy =  oy - map.height / 2;
	Fy =  oy + map.height / 2;

	oy = Math.round(oy / s) * s;
	for (i = -d; i <= d; i++) drawLine(c, fx, oy + i * s, Fx, oy + i * s, "gray");
	d = map.width / s / 2;
	ox = Math.round(ox / s) * s;
	for (i = -d; i <= d; i++) drawLine(c, ox + i * s, fy, ox + i * s, Fy, "gray");
	c.save();
	c.scale(1, -1);
	drawCartesian(c, 100 * scale, 100 * scale, 3);
	c.restore();
}

function center_robot() {
	tx = map.width / 2 - (rosy.x / 10) * scale;
	ty = map.height / 2 + (rosy.y / 10) * scale;
}

function draw(c) {
	if(rosy_refresh || rosy != rosy_next) {
		if(rosy != null) { 
			if(rosy.zed) rosy.zed.unsubscribe(); 
			if(rosy.lidar_front) rosy.lidar_front.lidarsubscriber.unsubscribe();
			rosy.zed = rosy.lidar_front = null;
		}
		if(rosy_next != null)	{
			if(document.getElementById('chkshowlidarfront').checked) rosy_next.lidar_front = new LiDAR(rosy_next.rosd, rosy_next.ns, "black", "scan");
		}
		rosy_refresh = false;
	}
	rosy = rosy_next;
	//map.width = map.parentElement.offsetWidth-100;
	map.width = window.innerWidth - 20;	
	map.height = window.innerHeight - map.parentElement.offsetTop - 20;
	c.save();
	c.clearRect(0, 0, map.width, map.height); //clear visible area
	c.translate(tx, ty);
	drawGrid(c);
	c.scale(scale, -scale);

	drawPoint(c, dest_x, dest_y, 2, 'magenta'); //destination

	if (rosy && rosy.pose.last_msg) {
		if (document.getElementById("chkrotate").checked) {
			c.rotate(-rosy.z);
		}
		rosy.draw(c);
		document.getElementById('batteryInfo').innerText = rosy.battery_voltage.value + 'V ';
		if (document.getElementById("chkfollow").checked) center_robot();
		document.getElementById("pose").innerHTML =
			"X: " + rosy.x + "mm Y:" + rosy.y + "mm Z:" + rosy.Z + "*";
		}

	c.restore();
}

function translate(dx, dy) {
	tx += dx;
	ty += dy;
}

window.onload = function () {	
	ros = new ROSBridge(robot_URL, true);
	document.getElementById("cbgs").value = grids; //combobox grid spacing
	document.getElementById("cbms").value = scale; //combobox map scale
	document.getElementById("cbps").value = points; //combobox map scale
	document.getElementById("map").onwheel = function (e) {
		e.preventDefault();
		c = document.getElementById("cbms");
		if (e.deltaY < 0 && c.selectedIndex > 0) {
			c.selectedIndex--;
		}
		if (e.deltaY > 0 && c.selectedIndex < c.options.length - 1) {
			c.selectedIndex++;
		}
		c.onchange();
	};

	state = document.getElementById('state');
	map = document.getElementById("map");
	tx = map.width / 2;
	ty = map.height / 2;
	const mapc = map.getContext("2d");
	setInterval(function () {
		draw(mapc);
	}, 50);

	map.addEventListener("mousedown", function (e) {
		mouseMoved = false
		mouse_buttons |= (1<<e.button);
		var mouse = crossBrowserRelativeMousePos(e);		
		if (e.button == 0) {
			lastMouseDownCoord = translateStart = crossBrowserRelativeMousePos(e);			
			now = new Date();
			interMouseDownMs = now - lastMouseDownTS;
			mouseIsPressed = true
			lastMouseDownTS = now;
		}
		if(e.button == 2) {
			rosy.cancelAction();
		}
	});

	map.addEventListener("mouseup", function (e) {
		mouse_buttons &= ~(1<<e.button);
		// console.log("up");
		if (e.button == 0) {
			mouseIsPressed = false
			if(interMouseDownMs>250) {
				var mouse = crossBrowserRelativeMousePos(e);
				translate(mouse.x - translateStart.x, mouse.y - translateStart.y);
			}
		}
	});

	map.addEventListener('dblclick', function (e) {
		mouse = crossBrowserRelativeMousePos(e);
		dest_dx = mouse.x - lastMouseDownCoord.x;
		dest_dy = mouse.y - lastMouseDownCoord.y;
		dest_d = Math.sqrt(dest_dx*dest_dx+dest_dy*dest_dy);
		console.log("dest_d", dest_d);
		dest_z = dest_d>20?Math.atan2(-dest_dy, dest_dx):10;
		dest_x = lastMouseDownCoord.x/scale;
		dest_y = -lastMouseDownCoord.y/scale;
		rosy.sendG1([dest_x / 100, dest_y / 100, dest_z]);

	});

	map.addEventListener("mousemove", function (e) {
		mouseMoved = true
		mouse_buttons &= ~(1<<e.button);
		currentMousePos = crossBrowserRelativeMousePos(e);
		if (e.button == 0 && mouseIsPressed && translateStart) {
			if(interMouseDownMs>250) {
				var mouse = crossBrowserRelativeMousePos(e);
				translate(mouse.x - translateStart.x , mouse.y - translateStart.y);
			}
		}
	});	
	zed = new DragableElement('zed');
	plc = new DragableIframe('plc');
	rosy_next = new Rosy(ros, '/r0/')
};