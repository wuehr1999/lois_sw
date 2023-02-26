class RobotParameters
{
	constructor()
	{
		this.ENCODER_STEPS = 60;
		this.INTERVAL_RPMCTRL_MS = 100;
		this.WHEEL_RADIUS = 0.125;
		this.VMAX = 1.5;
    this.RPMMAX = 100;
    console.log("Parameters loaded...");
	}
}
console.log("BOT");
var bot = new RobotParameters();
