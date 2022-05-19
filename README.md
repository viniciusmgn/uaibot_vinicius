

<h1>UAIBot</h1>

<h2>Introduction</h2>

<p align="center">
    <img src="https://viniciusmgn.github.io/aulas_manipuladores/presentation/images/aula1/logouaibot.svg" alt="UAIBot Logo"/>
</p>

UAIBot is a <strong>web-based Python robotic simulator</strong> developed by <strong>Vinicius Mariano Gonçalves</strong>  (Electrical Engineering Department, Federal University of Minas Gerais, Brazil) and his students. 

While teaching robotics, I used many different desktop-based simulators with my students (such as CoppeliaSim and Matlab Toolboxes). However, I realized that students nowadays are much more used to web-based applications. This is why I, together with my students, came with the idea of creating a simulator with the following goals:

<ul>
  <li>It can be used in a web-broser, if the student desires.</li>
  <li>Programming should be done in a language that most students already know or have some interested in learning, i.e, they should not be forced to learn a very specific language to use this simulator. Nowadays, the language that better fits these requirements is <strong>Python</strong>.  </li>
  <li>It should be easy to set up and simple to use.</li>
  <li>For didactic purposes, it should be a <strong>low-level</strong> simulator. This means that is up to the user to simulate everything, with the help of the functions/interfaces from the simulator. Since everything is under the user control, if something goes awry it is easier to pinpoint what is wrong.</li>
  
</ul>

Guided by these goals, me and my student [Johnata Brayan](http://setpointcapybara.com/site/) came, in January 2022,  with the idea of creating <strong>UAIBot</strong>.
It is focused, so far, in <strong>open-chain serial robotic manipulators</strong>, although there is some limited support already for other kind of robots.

<h2>How it works</h2>

A Python library is used to code everything. First, it is used to <strong>set up the scenario</strong> (robots and other objects). Then, it is up to the user to explicitly compute each object movement using the provided interfaces, creating <strong>animation frames</strong> for each one of them.  Then the user <strong>create the interactive animation</strong> as a HTML file, that can be shared and even embedded in a Web Page for didactic purposes.

So, in UAIBot, all the simulation is first created (the computations take place) and the animation is displayed!

Examples of HTML simulations made using UAIBot can be seen [here](https://viniciusmgn.github.io/aulas_manipuladores/presentation/images/aula1/democontrole1.html) , [here](https://viniciusmgn.github.io/aulas_manipuladores/presentation/images/aula5/anim9.html) and [here](https://viniciusmgn.github.io/aulas_manipuladores/presentation/images/aula1/democontrole2.html).

The animations are displayed using [Three.js](https://threejs.org/), in JavaScript. So the Python code automatically generates the JavaScript code to set up the animation that was coded using Python. In fact, UAIBot wraps in Python many of Three.js' functions, allowing us to use many of Three.js' features to visually customize the simulation.

<h2>Getting started</h2>

It is easier to start using UAIBot in a web browser. We will use [GoogleColab](https://colab.research.google.com/) since it allows us to run Python code in a web browser.

Open a new notebook. Now, we need to install UAIBot in the GoogleColab servers. This can be done by simply running the following commands:

```python
!git clone https://github.com/viniciusmgn/uaibot_vinicius.git
!pip install uaibot_vinicius/
```

After it is done, we test if it is working by running the following command

```python
import uaibot as ub

sim = ub.Demo.control_demo_1()
```

This will generate a simulation that was already pre-coded into UAIbot. It will return the simulation variable (sim) and automatically run the animation for you!

If you want to run the simulation again, you dont need to compute it again. Just run <strong>sim.run()</strong>.

Note that you will need to reinstall UAIBot every time you open GoogleColab, since the virtual machine created for you will be deleted.

<h2>Using in desktop-based IDE's</h2>

You can install the UAIBot package in a desktop-based IDE as Pycharm. Install using the terminal

```python
>>git clone https://github.com/viniciusmgn/uaibot_vinicius.git
>>pip install uaibot_vinicius/
```

The <strong>sim.run()</strong> may not work in some IDEs. In that case, you need to save the simulation as a HTML file:

```python
import uaibot as ub

sim = ub.Demo.control_demo_1()
sim.save('C:\\','test_uaibot')
```

This will sabe the file <strong>test_uaibot.html</strong> in your C: directory. You can then just open and visualize it. You can share just this file with your friends as well, it usually will be a small file. Since much of the information (as 3D models) are stored in a web server, <strong>in order to visualize the file an internet connecton is required</strong>.

<h2>How to use the simulator</h2>

Please see the [UAIBot Wiki](???).

If you know portuguese, you can also see my [Robotic Manipulator course](https://viniciusmgn.github.io/aulas_manipuladores), which uses UAIBot.

<h2>Why "UAIBot"?</h2>

"Uai" is an interjection commonly used by mineiros, that is, people who where born in the state of Minas Gerais, Brazil. It is one of the regional symbols of Minas Gerais. It is pronounced like the english "why" and has roughly the same meaning, used when mineiros are confused or in doubt. Indeed, some linguistic researchers think that the origin of this interjection is exactly the english word "why".

<h2>What is exactly the logo of "UAIBot"???</h2>

It is supposed to be a robotic manipulator in front of a mountain. Mountains, along with aforementioned "Uai", are one of the symbols of the state of Minas Gerais, Brazil. 

<h2>Collaborators</h2>

[Johnata Brayan](http://setpointcapybara.com/site/) (Electrical Engineering student, UFMG)






