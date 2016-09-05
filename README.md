# ros-psoc
Rosserial-based embedded systems for the Cypress PSoC4 and PSoC5 families

## Device overview
These ARM-architecture Cypress processors are available in very low-cost "prototyping kit" boards
* PSoC4 [CY8CKIT-049](http://www.cypress.com/documentation/development-kitsboards/psoc-4-cy8ckit-049-4xxx-prototyping-kits) for under $5
* PSoC5 [CY8CKIT-059](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-059-psoc-5lp-prototyping-kit-onboard-programmer-and) for under $10

These boards are available through many retail distributors (e.g. DigiKey, Mouser)

## Development environment
The "Programmable System on a Chip" (PSoC) architecture supports highly configurable peripherals, including high-level (e.g. UART) and low-level (e.g. gate-level FPGA) components. Cypress provides a free, Windows-based graphical development tool ([PSoC Creator](http://www.cypress.com/products/psoc-creator-integrated-design-environment-ide)) whereby the design is entered in block-diagram/schematic form and a "fitter" allocates hardware resources to implement the design.

In addition, PSoC Creator is a complete IDE with program editor, compiler, linker, and download capability; it uses the arm-gcc toolchain. Developers who prefer to do code development in a Linux environment may do so using an "export" facility. This may be convenient, as ROS development and execution requires a Linux host. In any case, the initial step of configuring the peripheral "hardware" components is only practical in the PSoC Creator tool and this must run in a native Windows environment.

The files in this repository are intended to make up a PSoC Creator workspace. The repository can be cloned to any convenient location; in PSoC Creator navigate to and open the ros-psoc/ros-psoc.cywrk workspace.

##Status
At this time the PSoC4 project is functional; the PSoC5 project is under development.

##Licenses
Much of the code here is offered, like ROS itself, under a "permissive" open source license, such as BSD. This allows derivative works to be made proprietary, as commercial entities sometimes prefer. There is no guarantee that ALL code here is free of "GPL"-like or other restrictions; if this is important to you, you must do your own due diligence.

##Historical notes
The version of Rosserial here was forked from upstream [ros-drivers](https://github.com/ros-drivers/rosserial)sometime early in 2014 and adapted to the PSoC4 ARM environment. It includes some bug fixes and optimizations aimed at minimizing the code footprint (the PSoC4 has only 32K flash). It would be desirable to push the psoc codebase back upstream but it is not really in a clean enough shape to do that right now.

A version was posted on the Jigsaw Renaissance wiki (now defunct) as "http://wiki.jigren.org/index.php?title=rosserial_on_cypress_psoc4".

Subsequently, the code was used in development of a prototype turbidistat at HiveBio in Seattle. This project has since migrated to [soundbio](https://github.com/soundbio/ministat-1). The soundbio project was forked to create this repository. Since the fork, the intent has been to go back to the idea of a "demo" repository that includes examples of many features, rather than addressing a specific application.


##Licenses
Much of the code here is offered under a "permissive" open source license, like BSD. 




