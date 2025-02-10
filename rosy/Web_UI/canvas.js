var publishImmidiately=true;
var manager = null;
function createJoystick(moveAction) {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystickt');
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        managert = nipplejs.create({
            zone: joystickContainer,
            position: { left: 30 + '%', top: (joystickContainer.top) },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true,
            shape: 'square'
        });
        // event listener for joystick move
        managert.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var x = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var y = Math.sin(direction / 57.29) * nipple.distance * 0.005;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(x, y, 0);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        managert.on('end', function () {
            moveAction(0, 0, 0);
        });
        joystickContainer = document.getElementById('joystick');
        manager = nipplejs.create({
            zone: joystickContainer,
            position: { left: 70 + '%', top: (joystickContainer.top) },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        });
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, 0, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0, 0);
        });
    }
}
function drawLine(c, sx, sy, ex, ey, col, thick=1) {
    //c.fillStyle = col; 
    c.strokeStyle = col;
    c.lineWidth = thick; 
    c.beginPath(); 
    c.moveTo(sx, sy);
    c.lineTo(ex, ey);
    c.stroke();
}
function drawCircle(c, x, y, r, col) {
    c.beginPath();
    c.arc(x, y, r, 0, Math.PI*2);
    c.strokeStyle = col;
    c.lineWidth = 1;
    c.stroke();
}
function drawArcArrow(c, x, y, r, col,thick) {
    c.beginPath();
    c.strokeStyle = col;
    c.lineWidth = 1;    
    drawLine(c, -r, 0, -r-3*r/20,10*r/20, col,thick);
    drawLine(c, -r, 0, -r+7*r/20,7*r/20, col,thick);    
    c.stroke();

    c.beginPath();
    c.arc(x, y, r, 0, Math.PI);
    c.stroke();
}
function drawPoint(c, x, y, r, col) {
    c.beginPath();
    c.arc(x, y, r, 0, Math.PI*2);
    c.fillStyle = col;
    c.fill();
}
function drawCartesian(c, RL, RW, thick=1) {
    drawLine(c, 0, 0, RL, 0, 'red',thick);
    drawLine(c, RL-10, 5, RL, 0, 'red',thick);
    drawLine(c, RL-10, -5, RL, 0, 'red',thick);
    drawLine(c, 0, 0, 0, RW, 'green',thick);
    drawLine(c, 5, RW-10, 0, RW, 'green',thick);
    drawLine(c, -5, RW-10, 0, RW, 'green',thick);
    drawArcArrow(c, 0, 0, RL/5, 'blue',thick);
}


function crossBrowserKey(e) {
	e = e || window.event;
	return e.which || e.keyCode;
}

function crossBrowserElementPos(e) {
	e = e || window.event;
	var obj = e.target || e.srcElement;
	var x = 0,
		y = 0;
	while (obj.offsetParent) {
		x += obj.offsetLeft;
		y += obj.offsetTop;
		obj = obj.offsetParent;
	}
	return { x: x, y: y };
}

function crossBrowserMousePos(e) {
	e = e || window.event;
	return {
		x:
			e.pageX ||
			e.clientX +
				document.body.scrollLeft +
				document.documentElement.scrollLeft,
		y:
			e.pageY ||
			e.clientY + document.body.scrollTop + document.documentElement.scrollTop,
	};
}
function crossBrowserRelativeMousePos(e) {
	var element = crossBrowserElementPos(e);
	var mouse = crossBrowserMousePos(e);
	return {
		x: mouse.x - element.x - tx,
		y: mouse.y - element.y - ty,
	};
}

class DragableElement {
	constructor(id) {
		var elem = document.getElementById('zed');
		var translateStartZed = null;
		elem.addEventListener("mousedown", function (e) {
			if (e.button == 0) {
				e.preventDefault();
				console.log('down');
				if (null == translateStartZed) {
					translateStartZed = crossBrowserRelativeMousePos(e);
					e.stopPropagation();
					return false;
				}
			}		
		});
		elem.addEventListener("mouseup", function (e) {
			if (e.button == 0) {
				translateStartZed = null;
			}
		});
		elem.addEventListener("mousemove", function (e) {
			if (translateStartZed != null) {
				var mouse = crossBrowserRelativeMousePos(e);
				elem.style.top = (parseInt(elem.style.top) + mouse.y - translateStartZed.y).toString()+"px";
				elem.style.left = (parseInt(elem.style.left) + mouse.x - translateStartZed.x).toString()+"px";
				e.stopPropagation();
				return false;
			}
		});
		elem.onwheel = function (e) {
			e.preventDefault();
			var c = parseInt(elem.style.width);
			if (e.deltaY < 0 && c > 100) {
				elem.style.width = (c-100).toString() + 'px';
			}
			if (e.deltaY > 0 && c<2000) {
				elem.style.width = (c+100).toString() + 'px';
			}
		};
	}
}

class DragableIframe {
	constructor(divid) {
		this.docu = null;
		this.ifr = null;
		this.win = null;
		this.divid = divid;
		this.div = document.getElementById(this.divid);
		this.div.style = 'left: 500px; top: 500px;width:1064px;height:808px;background-color:#fefefe;display: none;color:white;border-radius: 10px;padding:17px;box-shadow: 5px 5px 10px 2px #888888;cursor:move;position:absolute;xposition:relative;'
		var selected = null, // Object of the element to be moved
				x_pos = 0, y_pos = 0, // Stores x & y coordinates of the mouse pointer
				x_elem = 0, y_elem = 0; // Stores top, left values (edge) of the element
		
		// Will be called when user starts dragging an element
		function _drag_init(elem) {
				// Store the object of the element which needs to be moved
				selected = elem;
				x_elem = x_pos - selected.offsetLeft;
				y_elem = y_pos - selected.offsetTop;
		}
		
		// Will be called when user dragging an element
		function _move_elem(e) {
				x_pos = document.all ? window.event.clientX : e.pageX;
				y_pos = document.all ? window.event.clientY : e.pageY;
				if (selected !== null) {
						selected.style.left = (x_pos - x_elem) + 'px';
						selected.style.top = (y_pos - y_elem) + 'px';
				}
		}
		
		// Destroy the object when we are done
		function _destroy() {
				selected = null;
		}
		
		// Bind the functions...
		this.div.onmousedown = function () {
				_drag_init(this);
				return false;
		};
		
		document.onmousemove = _move_elem;
		document.onmouseup = _destroy;
	}
	show(url, ifr_id='ifr_plc'){
			var myPop = document.getElementById(this.divid);
			myPop.innerHTML = "<iframe id='"+ifr_id+"' scrolling='no' style='border: 1px solid black;position:absolute;top:20px;left:20px;z-index:1; zoom: 1;-moz-transform: scale(1);-moz-transform-origin: 0 0;-o-transform: scale(1);-o-transform-origin: 0 0;-webkit-transform: scale(1);-webkit-transform-origin: 0 0;' width=1024 height=768 src='" + url + "'></iframe>";
			myPop.style.display = 'block';
			this.ifr = document.getElementById(ifr_id);
			if(this.ifr) this.win = this.ifr.contentWindow;
			if(this.docu) this.docu = this.win.document;
		}
	hide(){
			var myPop = document.getElementById(this.divid);
			myPop.innerHTML = "";
			myPop.style.display = 'none';
			this.docu = null
		}
}

class DragableIframe2 {
	constructor(divid) {
		this.docu = null;
		this.ifr = null;
		this.win = null;
		this.divid = divid;
		this.div = document.getElementById(this.divid);
		this.div.style = 'left: 500px; top: 500px;width:1064px;height:808px;background-color:#fefefe;display: none;color:white;border-radius: 10px;padding:17px;box-shadow: 5px 5px 10px 2px #888888;cursor:move;position:absolute;xposition:relative;'
		var selected = null, // Object of the element to be moved
				x_pos = 0, y_pos = 0, // Stores x & y coordinates of the mouse pointer
				x_elem = 0, y_elem = 0; // Stores top, left values (edge) of the element
		
		// Will be called when user starts dragging an element
		function _drag_init(elem) {
				// Store the object of the element which needs to be moved
				selected = elem;
				x_elem = x_pos - selected.offsetLeft;
				y_elem = y_pos - selected.offsetTop;
		}
		
		// Will be called when user dragging an element
		function _move_elem(e) {
				x_pos = document.all ? window.event.clientX : e.pageX;
				y_pos = document.all ? window.event.clientY : e.pageY;
				if (selected !== null) {
						selected.style.left = (x_pos - x_elem) + 'px';
						selected.style.top = (y_pos - y_elem) + 'px';
				}
		}
		
		// Destroy the object when we are done
		function _destroy() {
				selected = null;
		}
		
		// Bind the functions...
		this.div.onmousedown = function () {
				_drag_init(this);
				return false;
		};
		
		document.onmousemove = _move_elem;
		document.onmouseup = _destroy;
	}
	show(url, ifr_id='ifr_plc'){
			var myPop = document.getElementById(this.divid);
			myPop.innerHTML = "<iframe id='"+ifr_id+"' scrolling='no' style='border: 1px solid black;position:absolute;top:20px;left:20px;z-index:1; zoom: 1;-moz-transform: scale(1);-moz-transform-origin: 0 0;-o-transform: scale(1);-o-transform-origin: 0 0;-webkit-transform: scale(1);-webkit-transform-origin: 0 0;' width=1024 height=768 src='" + url + "'></iframe>";
			myPop.style.display = 'block';
			this.ifr = document.getElementById(ifr_id);
			if(this.ifr) this.win = this.ifr.contentWindow;
			if(this.docu) this.docu = this.win.document;
		}
	hide(){
			var myPop = document.getElementById(this.divid);
			myPop.innerHTML = "";
			myPop.style.display = 'none';
			this.docu = null
		}
}