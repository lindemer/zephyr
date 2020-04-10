.. _hello_world:

Hello World
###########

Overview
********
A simple Hello World example that can be used with any supported board and
prints 'Hello World' to the console. This application can be built into modes:

* single thread
* multi threading

Building and Running
********************

This project outputs 'Hello World' to the console.  It can be built and executed
on QEMU as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :host-os: unix
   :board: qemu_riscv32
   :goals: run
   :compact:

Sample Output
=============

.. code-block:: console

    Hello World from UserSpace! qemu_riscv32

Exit QEMU by pressing :kbd:`CTRL+A` :kbd:`x`.
