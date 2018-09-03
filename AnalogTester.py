# Libraries
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Software SPI
# The library interface uses BCM labeling
# Connect the chip to the following pins
#       CLK to GPIO 18 (physical pin 12)
#       MISO to GPIO 23 (physical pin 16)
#       MOSI to GPIO 24 (physical pin 18)
#       CS to GPIO 25 (physical pin 22) 
mcp = Adafruit_MCP3008.MCP3008(clk=18,cs=25,miso=23,mosi=24)


print("Reading values")

# Main program 
if __name__ == '__main__':
    try:
        
        while True:

            values = [0]*8
            for i in range(8):
                values[i] = mcp.read_adc(i)

            print(' | {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
            time.sleep(0.5)
            
            
    
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
