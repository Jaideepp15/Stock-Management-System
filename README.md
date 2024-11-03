Stock management System using STM32F411 in keil microvision 5.
Components:-
1) STM32F411
2) HC05 BLUETOOTH MODULE
3) 1 Kg load cell
4) HX711 Amplifier
5) Two LEDS

This project treats the readings from the load cell as the weight of a section in a supermarket and displays the stock status along with the weight in the owner's phone.
Status includes 'In stock','Only few in stock' and 'Out of stock'.
Additionally if the owner types 'O' in the bluetooth serial contoller mobile application he/she can view the amount of weight that has to be ordered.
In the hardware the red led is turned on when there is very few or no items in stock and the green led turns on when the items are in stock.
The load cell gives he weight in analog form which is amplified and converted to digital via the hx711 amplifier.
Weight is measured whenver an interrupt is signalled by the hx711 amplifier throught the nested vector interrupt controller.
