# Practical 2: Position-Based Dynamics

##Handout date: 21/Mar/2017.

##Deadline: 4/Apr/2017 09:00AM.

*Note: demos have been added to the `demo` subfolder.*

The second practical will be about simulating systems of particles that are attached together by constraints, within the position-based framework. The objective of the practical are:

1. Implement sequential position-based constraint resolution.
 <br />
2. Integrate and couple velocities and positions.
 <br />
3. Extend the framework with some chosen effects.  

This is the repository for the skeleton on which you will build your practical. Using CMake allows you to work and submit your code in all platforms. This is practically the same system as in the previous practical, in terms of compilation and setup.

<span style="color:blue">**Note: please read through all instructions and the code skeleton carefully if you get stuck, and ask questions in case something is unclear. Often the answer is written down already</span>**.


##Scope

The practical runs in the same time loop (that can also be run step-by-step) as the previous practical. The objects are not limited to convex ones; every triangle mesh would do. The software converts the mesh into a set of spherical particles which are vertex based, and draws constraints between them to enforce rigidity, collisions, and auxiliary constraints. There are no ambient forces but gravity in the basic version. Note that the spherical particles only approximate the mesh; you might see collisions which are not exact triangle-to-triangle, and that is acceptable. Moreover, there might be some penetration on edges if the mesh is not dense enough.

A considerable difference is that the platform is no longer a mesh in the usual sense (although it appears as such)---it is treated as a barrier constraint, where objects cannot fall beyond it. Particles are only assigned to loaded meshes.

The major difference from previous methods is that each particle is a dynamic entity on its own: it has a position (center) $x_i$, a velocity $v_i$, an inverse mass $\omega_i$, and a radius $r_i$. The mesh rigid (or otherwise) movement is implicity by enforcing rigidity constraints. That simplifies collision detection and resolution considerably, but as you will see, is also less accurate physically.

The practical is configured to handle four possible constraints:

1. **Barrier**: one coordinate of the object is bounded. For instance, the natural constraint for every particle on the platform of height $\frac{h}{2}$ is $C_B=y - \frac{h}{2} \geq 0$.
<br />
2. **Collision**: two spherical particles penetrate each other. The constraint is then the inequality $C_C = \left|\vec{x}_{ij}\right|-(r_i+r_j)\geq 0$, where $\vec{x}_{ij}=\vec{x}_i-\vec{x}_j$ is the vector between the two centers.
<br />
3. **Rigidity**: The particles of the mesh which share an edge $\vec{e}_{ij}$ also produce a constraint $C_e = \left|\vec{x}_{ij}\right|-\left|\vec{e}_{ij}\right|=0$, where $\left|\vec{e}_{ij}\right|$ is the original edge length. This constraint makes sure the distance is kept while moving throuh space, and does not cause deformation.
<br />
4. **Attachment**: a user-prescribed constraint couples the coordinates of one vertex of one mesh with another. That is, the constraint $C_A=\vec{x}_i-\vec{x}_j-\vec{a}_{ij}=0$, where $\vec{a}_{ij}$ is the original vector between them both. Note that this does not allow rotation (this is hanndled as an extension). If $\vec{a}_{ij}=0$ originally, then the objects keep attaching. Note that we do not do vector constraints. Each constraint is a single scalar function. Therefore, attachment constraints are actually encoded as 3 independent constraints: one per coordinate.

###Constraints and gradients

A single constraint is usually only expressed as the function of the coordinates of two particles (that means mostly up to  variables). You need to analytically compute the gradient of these functions, and apply them in the constraint code (below explained how). Example: the rigidity constraint: $C_e = \left|\vec{x}_{ij}\right|-\left|\vec{e}_{ij}\right|=0$, and we get:

$$\nabla C_e = \left( \frac{\vec{x}_{ij}}{\left| \vec{x}_{ij} \right|}, -\frac{\vec{x}_{ij}}{\left| \vec{x}_{ij} \right|} \right)$$

Where the gradient expressed in the 6 varibles of the three coordinates to $\vec{x}_i$ and $\vec{x}_j$ each.

The other constraints have mostly straightforward gradients. Note that the barrier constraint, for instance, only applies to a single coordinate in a vertex. This is made possible due to the indexing scheme we apply in this practicle.


###Basic Requirements

The practical includes the following basic mandatory requirements:


1. For every time step, integrate the accelerations and add the impulses of particles into velocities, and integrate the velocities into positions. Use an *semi-implicit scheme*, as learned in class(Lecture 5). Note that this just means to integrate velocity to $v(t+\Delta t)$ and subsequently position from the integrated velocity. The time step difference $\Delta t$ is given by the GUI, and controllable by the menu.
<br />
2. For every time step, construct the collision, rigidity, attachment, and barrier constraints, and resolve them by projecting the integrated positions one by one until the entire system is satisfied. 
<br />
3. For every time step, update the velocities to match the positions (the position-based step).
<br />
4. Apply impulses from collision to change the velocities abruptly. This is a hack, but it works reasonable: for every position change $\Delta x$ induced by correcting a constraint, create impulse to a velocity change of $CR \cdot \frac{\Delta x}{\Delta t}$. Remember that impulses are only aggregated at this stage, and integrated in step (1) of this algorithm.


See below for details on where to do all that in the code.

###Extensions

The above will earn you $70\%$ of the grade. To get a full $100$, you must choose 2 of these 5 extension options, and augment the practical. Some will require minor adaptations to the GUI or the function structure which are easy to do. Each extension will earn you $15\%$, and the exact grading will commensurate with the difficulty. Note that this means that all extensions are equal in grade; if you take on a hard extension, it's your own challenge to complete well.

1. Enhance the attachment constraint to include full rigidity. **Level: very easy**.
 <br />
2. Change the rigidity constraint to become an inequality, where a rigidity value controls the ratio of the edge length that can be extended\compressed. This will make objects more plastic and susceptible to deformation. **Level: intermediate**. 
 <br />
3. Allow to apply random user-prescribed impulses to objects, or constant forces (like wind). This requires modest modifications to the GUI. **Level: easy-intermediate**. 
 <br />
4. Implement rigid-body projection by finding the rigid transformation from the original, as learnt in class (Lecture 9) rather than stiffen all edges sequentially. This should be exemplified by a scene that clearly shows the speed-up. **Level: intermediate**.
 <br />
5. Resolve interlocking, instead of naive particle constraints, as learnt in Lecture 9. You will probably want to read the following paper: [Unified Particle Physics for Real-Time Applications](http://mmacklin.com/uppfrta_preprint.pdf). It is anyhow a recommended read to understand the particle business. **Level: hard**
 <br />

You may invent your own extension as substitute to **one** in the list above, but it needs approval on the Lecturer's behalf **beforehand**.


##Installation

The installation follows the exact path as the previous practical. It is repeated for completeness.

The skeleton uses the following dependencies: [libigl](http://libigl.github.io/libigl/), and consequently [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), for the representation and viewing of geometry, and [libccd](https://github.com/danfis/libccd) for collision detection. libigl viewer is using [nanogui](https://github.com/wjakob/nanogui) for the menu. Everything is bundled as either submodules, or just incorporated code within the environment, and you do not have to take care of any installation details. To get the library, use:

```bash
git clone --recursive https://github.com/avaxman/INFOMGP-Practical2.git
```

to compile the environment, go into the `practical2` folder and enter in a terminal (macOS/Linux):

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

In windows, you need to use [cmake-gui](https://cmake.org/runningcmake/). Pressing twice ``configure`` and then ``generate`` will generate a Visual Studio solution in which you can work. The active soution should be ``practical2_bin``. *Note*: it only seems to work in 64-bit mode. 32-bit mode might give alignment errors.

##Working with the repository

All the code you need to update is in the ``practical2`` folder. Please do not attempt to commit any changes to the repository. <span style="color:red">You may ONLY fork the repository for your convenience and work on it if you can somehow make the forked repository PRIVATE afterwards</span>. Solutions to the practical which are published online in any public manner will **disqualify** the students! submission will be done in the "classical" department style of submission servers, published separately.

##The coding environment for the tasks

Most of the action, and where you will have to write code, happens in `scene.h` and `constraint.h`. The functions in `scene.h` implement the time loop and the algorithmic steps, while `constraint.h` implements the `Constraint` class that evaluates constraint and how to resolve them. 

The code you have to complete is always marked as:

```cpp
/***************
TODO
***************/
```

For the most part, the description of the function will tell you what exactly you need to put in.

###Input

The program is loaded by giving a TXT file that describes the scene as an argument to the executable. It is similar to the format in practical1, but with extensions. That means that there is no backward compatibility; you will have to update files you already made (a bit)

The file should be in the `data` subfolder, which is automatically discovered by the CMake. The format of the file is:

```
#num_objects #num_attachments
object1.off     density1    rigidity1   is_fixed1    COM1     o1
object2.off     density2    rigidity2   is_fixed2    COM2     o2 
.....

m1 v1 m2 v2
.....
```

Where:

1. ``objectX.off`` - an OFF file (automatically assumed in the `data` subfolder) describing the geometry of the a triangulated mesh. [Meshlab](http://www.meshlab.net/) can view these files, but their format is pretty straightforward. The object is loaded, and translated to have a center of mass of $(0,0,0)$ in its object coordinates.
<br />
2. ``density`` - the uniform density of the object. The program will automatically compute the mass by the volume.
<br />
2. ``rigidity`` - the rigidity of the object. This is only used in extension (2) where is is decided how to resolve constraints.
<br />
3. ``is_fixed`` - if the object should be immobile (fixed in space) or not.
<br />
4. ``COM`` - the initial position in space where the object would be translated to. That means, where the COM is at time $t=0$.
<br />
5. ``o`` - the initial orientation of the object, expressed as a quaternion that rotates the geometry to $o*object*inv(o)$ at time $t=0$.

The file describes a set of attachment constraints as follows: every line `m1 v1 m2 v2` adds an attachment constraint for each coordinate (so 3 constraints in total per line) between vertex `v1` in mesh `m1` and vertex `v2` in mesh `v2`. The indices are mesh-relative, not raw (see below)



###Indexing

Since every mesh has its own data structure for vertices etc., but we have inter-mesh constraints like attachment, and moreover since we have constraints that pertain to only single coordinates, we have two indexing methods:

1. **Regular indexing**: the one you already know. Vertices are kept in an $\left| V \right| \times 3$ matrix, and the vertex index refers to the row representing them. This is used for the triangles $T$, visualization, working inside the mesh, and specifying the attachment constraints in the input file.
<br />
2. **Raw indexing**: this is the indexing used by the scene class, which aggregates all particles in the scene, and also encoded like this in a constraints class. This is indexing a flat single column vector of **all** particles in the scene arranged in $xyzxyzxyz...$ order (so if there are $n$ particles in total, this is a $3n$ column vector). This applies to velocities, impulses, inverse masses, and radii as well, in the same indexing and totally compatible to each other, as part of the particle state. This column vectors are usually called, e.g., `rawVel`, to indicate they are raw-indexed.

this aggregates all vertices of all meshes by order. Two functions: `updateRawState()` and `updateMeshState()` update the raw data in the scene vs. the mesh data. the `rawOffset` variable in each mesh is the index in the global raw column where the $x$ coordinate of its first vertex should be (or the first velocity component etc. in the raw velocity column etc.). For the most part, you don't need to mess with this, unless you are creating new constraints.

###Data structure

There are three main classes, `Scene`, `Mesh`, and `Constraint`. They are commented throughout, so their individual properties are understood. Each mesh, and the platform, are bodies of their own. The geometry is encoded as follows in the `Mesh` class (note: previous practical used `origV` to mention positions, and we now use `origX` throughout, to match the lecture slide notations better):

```cpp
MatrixXd origX;   //original particle positions - never change this!
MatrixXd prevX;   //the previous time step positions
MatrixXd currX;   //current particle positions
MatrixXi T;       //the triangles of the original mesh
MatrixXd currNormals;
VectorXd invMasses;   //inverse masses of particles, computed (autmatically) as 1.0/(density * particle area)
VectorXd radii;      //radii of particles
int rawOffset;  //the raw index offset of the x value of V from the beginning of the 3*|V| particles
    
    
//kinematics
bool isFixed;  //is the object immobile
double rigidity;  //how much the mesh is really rigid
MatrixXd currVel;     //velocities per particle. Exactly in the size of origV
vector<Constraint> meshConstraints;  //the systematic constraints in the mesh (i.e., rigidity)
MatrixXd currImpulses;
```

The code is self-explanatory. `prevX` refers to the positions from the previous time step, and `currX` is the one currently updated. see explanation for `rawOffset` above.

A single constraint is encoded as follows in the `Constraint` class:

```cpp
VectorXi particleIndices;  //list of participating indices
double currValue;                       //the current value of the constraint
VectorXd currGradient;               //practicleIndices-sized.
MatrixXd invMassMatrix;              //M^{-1} matrix
VectorXd radii;
double refValue;                    //reference value to compare against. Can be rest length, dihedral angle, barrier, etc.
double stiffness;
ConstraintType constraintType;  //the type of the constraint, and will affect the value and the gradient. This SHOULD NOT change after initialization!
```

The ``particleIndices`` is used for the scene class to indicate what do the local variables in the ``Constraint`` class link to in the raw indexing. The constraint class itself only has the local constraint variables, and never has to refer to this. If the constraint hase $m$ participating variables (example: a rigidity constraint has 6 variables), ``currValue`` is the scalar value of the constraint, and ``currGradient`` is the $m$ vector of partial derivative values. the invMassMatrix is a diagonal matrix of all the inverse masses by the same order. ``refValue`` indicates some external constant scalar value used by the constraint. For instance, the edge length. The ``constraintType`` should be set upon initialization.




##Submission

The entire code of the practical has to be submitted in a zip file to the designated submission server that will be anounced. The deadline is **4/Apr/2017 09:00AM**. 

The practical must be done **in pairs**. Doing it alone requires a-priori permission. Any other combination (more than 2 people, or some fractional number) is not allowed. 

The practical will be checked during the lecture time of the deadline date. Every pair will have 10 minutes to shortly present their practical, and be tested by the lecturer with some fresh scene files. In addition, the lecturer will ask every person a short question that should be easy to answer if this person was fully involved in the exercise. We might need more than the allocated two hours for the lecture time; you will very soon receive a notification. The registration for time slots will appear soon.

<span style="color:red">**Very important:**</span> come to the submission with a compiled and working exercise on your own computers. If you cannot do so, tell the lecturer **in advance**. Do not put the command-line arguments in code, so that we waste time on compilation with every scene (is there any logic in doing so? it was common for some reason in the submissions).

##Frequently Asked Questions

Here are detailed answers to common questions. Please read through whenever you have a problem, since in most cases someone else would have had it as well.

<span style="color:blue">Q: </span> How do I apply impulses to collisions, as in the first practical? it's never really mentioned in the slides.
<span style="color:blue">A:</span> It is indeed not, since this is not an inherent part of position-based dynamics. What you should do in this practical is an addition described in the algorithm in these instruction, and referred to in the code when needed, of adding impulses upon position fixing.

<span style="color:blue">Q: </span> The demo doesn't run like the exercise.
<span style="color:blue">A:</span> The demo needs also the DATA folder as the input, because of some compilation issues; see readme in the `demo` subfolder. It was the same in practical 1.

<span style="color:blue">Q: </span> Are `currNormals` used for collision solving and impulse giving?
<span style="color:blue">A:</span> They are only used for the sophisticated collision resolution (extension 5). Otherwise, the supposed collision normal is implicitly given because the collision resolution just moves the objects in the direction of pushing the particle centers away. The resulting $\Delta x$ would be the normal (but this is not handled as a specific case, as this is not an explicit rigid-body simulation; it is just the result of the position-based approach).


<span style="color:blue">Q: </span> How do we "translate" the constraint formulas we have for each constraint to currGradient and currValue
<span style="color:blue">A:</span> currValue is a scalar wth the value of the constraint. currGradient is an $n$ vector, if $n$ are the participating variables, and each entry $i$ of the vector is $\frac{\partial C}{\partial p_i}$, where $p_i$ is the participating variable. See lengthy discussion above. Remember that a position is 3 different $x,y,z$ variables.





#Good work!











