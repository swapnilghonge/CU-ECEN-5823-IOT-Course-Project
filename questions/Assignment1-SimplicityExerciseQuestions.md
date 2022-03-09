Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: 5.22mA


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: 5.20mA


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: No, both values are approximately equal as expected. When the WeakAlternateWeak Drive strength is selected the instantaneous current measurement for the LED on was 5.20mA and when the LED was off was 4.57mA. This means that the current flowing through the LED is 5.20 - 4.57 = 0.63mA. As per the schematic both LED 0 & 1 are powered by a 3.3V supply with a resistance of 3k. Upon calculating the theoretical value for the current it comes to 0.5mA assuming the forward voltage of the LED is 1.6V. This is approximately within the expected value and the difference can be explained, as the accuracy of the AEM when current is more than 250uA is 0.1mA. (this calculation also holds true for StrongAlternateStrong)
   The drive strength is the maximum capacity of current a GPIO pin can drive. But for both weak and strong drive strength the current requirement for the LEDs are the same. Hence the GPIO pin will only drive 0.5mA for the LEDs irrespective of the drive strength. 
    The only meaningful difference in the current is the time it takes to go from low to high state or in other words the slew rate. For the strong drive strength, the GPIO pin can drive a current of 10mA so the ramp is shorter as compared to the weak drive strength where the GPIO can only drive 1mA. Hence, the slew rate for the StrongAlternateStrong drive strength is higher than WeakAlternateWeak.


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: 4.90mA


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: 5.08mA


