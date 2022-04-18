# ANNRA - Artificial Neural Network Robotics Arm

In the field of robotic control, artificial intelligence has become the most investigated modern technology. To implement artificial
intelligence for robots correctly, it’s needed to have a trustable model of kinematics and dynamics. In this research, a 3-DOF
sample of the industrial manipulator based on the ABB IRB 4400 model has been studied. Robot kinematics have been solved.
For forward kinematics, the method uses the DH-parameter, and for inverse kinematics, it uses the geometric approach to get the
end effector's final position. including with path-planning and path smoothing. For robot dynamics, the forward dynamics for
controlling robot arm joints using feedforward control with a PI controller. while the inverse dynamic equation is derived by using
Lagrange-Euler. Artificial neural networks (ANNs) solve the joint torque in a continuous path efficiently, overcoming the
drawbacks of inverse dynamics. The innovative controller architecture outperforms prior strategies in terms of lowering position
error in unusual situations and improving ANN accuracy in estimating robot joint angles. According to additional research, the
use of artificial neural networks to inverse dynamics might considerably improve the performance of a 6-DOF robotics arm or a
hyper-redundant manipulator in the future.

ได้ออกแบบหุ่นยนต์แขนกล 6 Degree of Freedom จากสมการ Kinematics และ Dynamics โดยจะนำไปทำเป็นหุ่นยนต์วาดรูปได้ตามต้องการ (Drawing Robot) และ ยังเป็นส่วนหนึ่งของคลิป SuperAI2-2811 “[ทำยังไงถึงจะรอดใน Squid Game? (ด้วยหุ่นยนต์ และ AI)](https://www.youtube.com/watch?v=rd8h8sUJEe8&t=7s)” 

Full paper [Inverse Dynamics Based Artificial Neural Network BFGS quasi-Newton backpropagation of 3 DOF Robotic Arm (IRB4400)](https://github.com/huak95/ANNRA/blob/main/Robotics_Arm_ANNs_full_paper.pdf)

![image](https://user-images.githubusercontent.com/38836072/163752505-b372ab42-f7cb-4bf6-bd88-717b0a918b6e.png)
![Slide2](https://user-images.githubusercontent.com/38836072/163752776-c89f0c79-f799-4c40-aed8-951547a8bc22.PNG)
![Slide3](https://user-images.githubusercontent.com/38836072/163752783-b0252e1c-276b-46f3-a099-ddbebb9affbd.PNG)
![Slide4](https://user-images.githubusercontent.com/38836072/163752786-d703c649-927c-4c56-8913-175f2f302093.PNG)
![Slide5](https://user-images.githubusercontent.com/38836072/163752825-c9c78f50-0101-4ef7-a3ea-55038411a0b9.PNG)
![Slide6](https://user-images.githubusercontent.com/38836072/163752828-0295b931-29a2-4317-97b1-54eadd31d7bf.PNG)
![Slide7](https://user-images.githubusercontent.com/38836072/163752829-62aecb07-9863-4381-bd81-5548ca88e2b6.PNG)
![Slide8](https://user-images.githubusercontent.com/38836072/163752831-45007e61-83a1-4042-b4c8-13301fd99ad1.PNG)
![Slide9](https://user-images.githubusercontent.com/38836072/163752832-9aa5740c-44d1-4996-a357-35ae53a7d157.PNG)
![Slide10](https://user-images.githubusercontent.com/38836072/163752814-37de0d41-878d-49c7-ae43-433dcb015f66.PNG)
![Slide11](https://user-images.githubusercontent.com/38836072/163752817-5710f0e4-e41d-4d2c-93a5-a7483faf13ea.PNG)
![Slide12](https://user-images.githubusercontent.com/38836072/163752818-081ab852-9d19-4bb9-8f21-d53f9fc6b078.PNG)
![Slide13](https://user-images.githubusercontent.com/38836072/163752820-95413b23-dac8-478d-8948-c23e121b3ba6.PNG)
![Slide14](https://user-images.githubusercontent.com/38836072/163752822-3ad5fb5b-19ef-4f39-915e-510e0a48a407.PNG)
![Slide15](https://user-images.githubusercontent.com/38836072/163752873-a7a2fed7-9216-4ca5-a150-70293e60fb31.PNG)
![Slide19](https://user-images.githubusercontent.com/38836072/163752880-58975a11-6bd6-441a-8fa0-12308400edbf.PNG)
![Slide21](https://user-images.githubusercontent.com/38836072/163752881-11e7e768-bbe9-4f90-9b93-261a68fc827b.PNG)
![Slide22](https://user-images.githubusercontent.com/38836072/163752883-fa478c5e-591f-4118-a0b2-f0f4fd8a6a17.PNG)
![Slide23](https://user-images.githubusercontent.com/38836072/163752885-01b6d2e9-4ae4-49d0-a08d-6f49d9d73570.PNG)
