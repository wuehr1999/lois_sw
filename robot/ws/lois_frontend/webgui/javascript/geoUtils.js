class Trackpoint
{
	constructor(lat, lon, altitude, speed, time)
	{
		this.lat = lat;
		this.lon = lon;
		this.altitude = altitude;
		this.speed = speed;
		this.time = time;
	}

	calculateDistance(point)
	{
		var la1 = this.lat;
		var lo1 = this.lon;
		var la2 = point.lat;
		var lo2 = point.lon;
		var latitude1R = (la1 * Math.PI) / 180.0;
		var longitude1R = (lo1 * Math.PI) / 180.0;
		var latitude2R = (la2 * Math.PI) / 180.0;
		var longitude2R = (lo2 * Math.PI) / 180.0;
		var dlat = latitude2R - latitude1R;
		var dlong = longitude2R - longitude1R;
		var a = Math.sin(dlat / 2.0) * Math.sin(dlat / 2.0) +
			Math.cos(latitude1R) * Math.cos(latitude2R) *
			Math.sin(dlong/ 2.0) * Math.sin(dlong / 2.0);	
		return 6371000.0 * 2.0 * Math.atan2(Math.sqrt(a), Math.sqrt(1.0 - a));	
	}

	calculateBearing(point)
	{
		var la1 = this.lat;
		var lo1 = this.lon;
		var la2 = point.lat;
		var lo2 = point.lon;
		var latitude1R = (la1 * Math.PI) / 180.0;
		var longitude1R = (lo1 * Math.PI) / 180.0;
		var latitude2R = (la2 * Math.PI) / 180.0;
		var longitude2R = (lo2 * Math.PI) / 180.0;

		var y = Math.sin(longitude2R - longitude1R) * Math.cos(latitude2R);
		var x = Math.cos(latitude1R) * Math.sin(latitude2R) -
			Math.sin(latitude1R) * Math.cos(latitude2R) * Math.cos(longitude2R - longitude1R);

		var bearing = Math.atan2(y, x) / Math.PI * 180.0;
		while(bearing < -180.0)
		{
			bearing += 360.0;
		}
		while(bearing > 180.0)
		{
			bearing -= 360.0;
		}
		return bearing;
	}

	xyPixelsFromStartPoint(startPoint, metersPerPixel)
	{
		var distance = startPoint.calculateDistance(new Trackpoint(this.lat, this.lon, 0, 0, 0));
		var bearing = startPoint.calculateBearing(new Trackpoint(this.lat, this.lon, 0, 0, 0)) - 90.0;
		var x = parseInt(Math.cos(bearing * Math.PI / 180.0) * distance / metersPerPixel);
		var y = parseInt(Math.sin(bearing * Math.PI / 180.0) * distance / metersPerPixel);

		return [x, y];
	}
}

class Track
{
	constructor()
	{
		this.points = [];
		this.minLat = 361;
		this.minLon = 361;
		this.maxLat = -361;
		this.maxLon = -361;
		this.maxHeight = 0.0;
		this.maxSpeed = 0.0;
		this.maxDist = 0.0;
	}

	add(trackpoint)
	{
		this.points.push(trackpoint)
		if(trackpoint.lat < this.minLat)
		{
			this.minLat = trackpoint.lat;
		}
		if(trackpoint.lat > this.maxLat)
		{
			this.maxLat = trackpoint.lat;
		}
		
		if(trackpoint.lon < this.minLon)
		{
			this.minLon = trackpoint.lon;
		}
		if(trackpoint.lon > this.maxLon)
		{
			this.maxLon = trackpoint.lon;
		}

		if(trackpoint.altitude > this.maxHeight)
		{
			this.maxHeight = trackpoint.altitude;
		}

		if(trackpoint.speed > this.maxSpeed)
		{
			this.maxSpeed = trackpoint.speed;
		}

		if(this.points.length > 1)
		{
			var tp1  = this.points[this.points.length - 2];
			var tp2 = this.points[this.points.length - 1];
	
			var dist = tp1.calculateDistance(tp2);	
			this.maxDist += dist;
		}
	}

	getNumberOfPoints()
	{
		return this.points.length;
	}

	getUpperLeft()
	{
		return new Trackpoint(this.maxLat, this.minLon, 0, 0, 0);
	}

	getLowerRight()
	{
		return new Trackpoint(this.minLat, this.maxLon, 0, 0, 0);
	}

}

