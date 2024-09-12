# Ant Method Math Explainer
The math for the ant method is a little weird.

**The Goal**
To make groups of colored LEDs (ants) of some size (g) progress down the strip
If we want groups of size 5 (g) we need to turn every 5th LED off. *Note the first LED is 0 so the groups will still be 5.*

To find if the current LED in the strip (i) is the 5th LED we subtract k (0) from i to get some number, which if it is the 5th LED it should be evenly divisible by 5 (g).
If (i - k) % g == 0 then the LED at i should be set to black.
If (i - k) % g != 0 then the LED at i should be set to the desired color, in this case red.
*Remember % is the modulus operator*
**But what is k? For now it is just 0, we get to it in a second.**

We first progress through every LED in the strip; every value of i.
The above method will turn every 5th LED off:
(0 - 0) % 5 == 0 and (5 - 0) % 5 == 0 but (2 - 0) % 5 != 0


But we want the black spaces to move to create the animation, this is where k comes in.
K is sort of the number of times we have moved the ant down the string, but it gets reset so not exactly.

To do this we must increment k after we have looped throught the whole LED strip
This makes it so that every 5th LED is still off but it is now one value of i farther down the strip.
For example: (1 - 1) % 5 == 0 and  (6 - 1) % 5 == 0 mean that LEDs 1 & 6 are black
but the previously black LED with i value of 5 will be red because (5 - 1) % 5 != 0 and i of 0 will also be black because (0 - 1) % 5 != 0.

**Remember that i of 1 is actually the second LED in the stip as 0 is the first**

We continue this looping through every value of i and then incrementing k until k is equal to g.
When k == g we can reset it to be 0 because (5 - 0) % 5 == 0 and (5 - 5) % 5 == 0 are the same thing.
This just helps to keep numbers small.

Now one final thing, what if we want more than one LED to be turned off.
Simple we create a b value which tells us how many LEDs behind the currently off LED should also be set to black.
It looks like this: if (i - k) % g == 0 then we will set the LEDs from i - b to i as black. This has the side effect of having to add b to g when you inpu them. 
If you want a group size of 5 and an off size of 5 as well you must make g 10.

And of course if you wanted two colors you could just replace setting the LEDs to black with any other color
