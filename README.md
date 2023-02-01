# SwitechX25 for STM32

Switech Library modified for use with the STM32 and re-written in C.

```c
static struct SwitecX25 *stepper;
stepper = init_stepper(Stepper_One_Pin, Stepper_Two_Pin,
                         Stepper_Three_Pin, Stepper_Four_Pin, GPIOB);

zero(stepper);
setPosition(stepper, 500);
updateBlocking(stepper);
```

Original:
[https://github.com/clearwater/SwitecX25](https://github.com/clearwater/SwitecX25)