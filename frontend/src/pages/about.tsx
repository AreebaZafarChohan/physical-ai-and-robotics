import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function About() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title="About RoboX" description="About the RoboX textbook">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="hero__title">About This Textbook</h1>
            <p>
              <b>Physical AI & Humanoid Robotics: A Textbook</b> is an open-source educational resource 
              that covers the fundamental concepts and practical applications of humanoid robotics. 
              This textbook is designed for students, researchers, and professionals interested in 
              the fields of robotics, artificial intelligence, and autonomous systems.
            </p>
            
            <h2>Mission</h2>
            <p>
              Our mission is to make advanced robotics education accessible to everyone. We believe 
              that humanoid robotics will play a crucial role in the future of human-robot interaction, 
              and we want to provide comprehensive resources to support learning and innovation in this field.
            </p>
            
            <h2>Content Overview</h2>
            <p>
              The textbook is organized into several modules that progressively build knowledge and skills:
            </p>
            <ul>
              <li><b>Module 1:</b> The Robotic Nervous System (ROS 2)</li>
              <li><b>Module 2:</b> The Digital Twin (Gazebo & Unity)</li>
              <li><b>Module 3:</b> The AI-Robot Brain (NVIDIA Isaac)</li>
              <li><b>Module 4:</b> Vision-Language-Action (VLA)</li>
              <li><b>Module 5:</b> Capstone Project</li>
            </ul>
            
            <h2>Contributing</h2>
            <p>
              This is an open-source project. We welcome contributions from the community. 
              Feel free to submit pull requests, report issues, or suggest improvements to the content.
            </p>
            
            <h2>License</h2>
            <p>
              The content of this textbook is licensed under Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0). 
              This means you are free to share and adapt the material, as long as you give appropriate credit and distribute 
              your contributions under the same license.
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default About;