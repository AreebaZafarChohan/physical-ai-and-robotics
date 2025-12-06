---
title: Quiz: Nodes, Topics, and Services in ROS 2
---

# Quiz: Nodes, Topics, and Services in ROS 2

Test your understanding of the core communication primitives in ROS 2.

---

## Question 1

Which ROS 2 primitive is best suited for continuous, one-way data streams like sensor readings or video feeds?

a) Node
b) Service
c) Topic
d) Action

**Answer**: c) Topic

---

## Question 2

A ROS 2 node is best described as:

a) A physical component of the robot.
b) A named bus for data exchange.
c) An executable process that performs computation.
d) A request-reply communication mechanism.

**Answer**: c) An executable process that performs computation.

---

## Question 3

In the context of ROS 2 services, what describes the client-server interaction?

a) Asynchronous data streaming.
b) Synchronous request/reply.
c) Continuous goal feedback.
d) Unidirectional message broadcasting.

**Answer**: b) Synchronous request/reply.

---

## Question 4

If multiple nodes are interested in receiving the same stream of data (e.g., robot's odometry), they would typically all be _____ to a common _____.

a) publishing, service
b) subscribing, topic
c) calling, action
d) advertising, node

**Answer**: b) subscribing, topic

---

## Question 5

Which of the following would be an appropriate use case for a ROS 2 service?

a) Constantly publishing robot joint states.
b) Sending a stream of camera images.
c) Requesting a robot to pick up a specific object and waiting for confirmation.
d) Estimating the robot's pose in real-time.

**Answer**: c) Requesting a robot to pick up a specific object and waiting for confirmation. (Actions are better for long-running, feedback-rich tasks like picking up, but a service could still be used for a simpler, blocking request.)
