Q) Summarize the project and what problem it was solving.

A) The project was to prototype a wireless thermostat. The system uses the temperature sensor to detect the room temperature and compare this to the setpoint. The setpoint is set by the user using two buttons on the board to increase or decrease the value. When the temperature is below the setpoint, the system would send a signal to turn on the heater. For this prototype, the heater was represented using an LED. To make the system more efficient, all tasks were controlled by a timer/task scheduler.


Q. What did you do particularly well?

A. For this particular project, the task scheduler was put together quite well. The button interrupts would store the change, and then apply them when the task was called to update the setpoint. This allowed for multiple buttons to be registered between the task being called instead of only one. 


Q. Where could you improve?

A. In this prototype, the 'heater' only has an on or off state. Where realistically it would have different options depending on how much of a difference in temperature. For example, it would be strange for the heater to go full power when there is only a single degree of temperature difference. If this project was to be updated, I would change the heater from an on/off state (0 or 1) to allow for values between 0 and 1. That value would be set by the observed temperature difference. Another quality update to be made would be to have a high and low setpoint. With a single value as the setpoint, the heater may turn on and off relatively rapidly as the temperature moves up and down around that degree. By allowing the setpoint, and then using a +/- range of 1 or 2 degrees, this can reduce how often the heater alternates states. So when the temperature is set to 70, the heater may not turn on until the temperature is at or below 68 or 69. Then it may not turn off until the temperature is above 71 or 72. 


Q. What tools and/or resources are you adding to your support network?

A. From this particular project, I found the embedded system manual to be incredibly helpful. I am now aware that any future embedded systems I work on will require me to seek out the technical documentation to best make use of the available hardware.


Q. What skills from this project will be particularly transferable to other projects and/or course work?

A. This was the first project where I needed to make use of interrupts and timers, and I found those aspects very beneficially. For systems that are meant to run for a long time or indefinitly, setting up a task scheduler and timer will be a great skill to have. 


Q. How did you make this project maintainable, readable, and adaptable?

A. This project included a lot of comments throughout the code, including the rationale behind some of the decisions. The variables used were chosen to make it easier to understand what the variable was for. An example would be the variable for the setpoint being 'setpoint' instead of a single character like 'x'. This, combined with the comments throughout the code, allow someone unfamiliar with the project to quickly understand it. This allows the code to be read easily and maintained. The project also makes use of various functions, that allow for it to be adapted without needing to change the entire code. 
