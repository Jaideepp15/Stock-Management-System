Stock management System using STM32F411 in keil microvision 5.
Components:-
1) STM32F411
2) HC05 BLUETOOTH MODULE
3) Force sensitive resistor
4) Two LEDS

This project treats the readings obtained from a force sensitive resistor as the weight of a section in a supermarket and displays the stock status along with the weight in the owner's phone.
Status includes 'In stock','Only few in stock' and 'Out of stock'.
Additionally When the owner types command 'O' in the bluetooth serial controller, an interrupt is generated and the weight to be ordered is calculated and displayed
In the hardware the red led is turned on when there is very few or no items in stock and the green led turns on when the items are in stock.
The load cell gives he weight in analog form which is converted to digital form using adc. The force sensitive resistor gives the difference in voltage whenever a force is apllied which is mapped to a weight using a linear relation.
