import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css';
import { motion } from 'framer-motion';

const HeroSection = () => (
  <section className={styles.hero}>
    <div className={styles.heroContent}>
      <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
      <p className={styles.heroSubtitle}>Designing Intelligence for the Physical World</p>
      <p className={styles.heroTagline}>Where Algorithms Gain Bodies and Machines Learn to Think</p>
      <div className={styles.heroButtons}>
        <a href="/docs/intro" className={styles.buttonPrimary}>Start Learning</a>
        <a href="/docs/intro" className={styles.buttonSecondary}>View Curriculum</a>
      </div>
      <a href="#" className={styles.syllabusLink}>Download Syllabus (PDF)</a>
    </div>
  </section>
);

const Section = ({ title, children, layout = 'normal' }) => (
    <section className={`${styles.section} ${layout === 'columns' ? styles.columns : ''}`}>
      <h2 className={styles.sectionTitle}>{title}</h2>
      {children}
    </section>
  );

const WhatIsPhysicalAI = () => (
    <Section title="What is Physical AI?" layout="columns">
      <div className={styles.column}>
        <p>Physical AI, or Embodied Intelligence, is the field of AI that gives algorithms a physical body, enabling them to perceive, reason, and act in the real world. It's the convergence of robotics, machine learning, and control theory to create intelligent agents that can physically interact with their environment.</p>
        <ul>
          <li>Multi-modal perception</li>
          <li>Real-world reasoning</li>
          <li>Closed-loop control</li>
          <li>Human-robot interaction</li>
        </ul>
      </div>
    
      <div className={styles.column}>
        <motion.div
          className={styles.motionDiagram}
          initial={{ opacity: 0, y: 40 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 1 }}
        >
          <motion.div
            className={styles.motionNode}
            animate={{ y: [0, -12, 0] }}
            transition={{ repeat: Infinity, duration: 2 }}
          >
            Sensors
          </motion.div>

          <motion.div
            className={styles.motionNode}
            animate={{ scale: [1, 1.1, 1] }}
            transition={{ repeat: Infinity, duration: 2 }}
          >
            AI Core
          </motion.div>

          <motion.div
            className={styles.motionNode}
            animate={{ x: [0, 12, 0] }}
            transition={{ repeat: Infinity, duration: 2 }}
          >
            Actuators
          </motion.div>
        </motion.div>
      </div>


    </Section>
  );
  
  const LearningOutcomes = () => (
    <Section title="Book Learning Outcomes">
      <div className={styles.cardGrid}>
        {[
          'Embodied intelligence foundations',
          'Robot perception & sensor fusion',
          'Manipulation & locomotion',
          'Humanoid robot design',
          'AI control systems',
          'Ethics and safety in physical AI',
        ].map((outcome, i) => (
          <div key={i} className={styles.card}>{outcome}</div>
        ))}
      </div>
    </Section>
  );
  
  const CurriculumRoadmap = () => (
    <Section title="Curriculum Roadmap">
      <div className={styles.timeline}>
        {[
          'Module 1: Foundation of Physical AI',
          'Module 2: ROS 2-Robotic Nervous System ',
          'Module 3: Digital Twin - Gazebo & Unity',
          'Module 4: AI Brain - NVIDIA Isaac',
          'Module 5: Vision-Language-Action-Systems',
          'Module 6: Capstone - Autonomous Humanoid',
        ].map((volume, i) => (
          <div key={i} className={styles.timelineItem}>{volume}</div>
        ))}
      </div>
    </Section>
  );
  
  const HandsOnPractice = () => (
    <Section title="Hands-On Practice">
      <div className={styles.iconGrid}>
        {[
          { icon: '🤖', text: 'Walking humanoid' },
          { icon: '🦾', text: 'Robotic grasping' },
          { icon: '👀', text: 'Vision-based navigation' },
        ].map((project, i) => (
          <div key={i} className={styles.iconItem}>
            <div className={styles.icon}>{project.icon}</div>
            <div>{project.text}</div>
          </div>
        ))}
      </div>
    </Section>
  );

  const TargetLearners = () => (
    <Section title="Target Learners">
      <div className={styles.cardGrid}>
        {['Student', 'Researcher', 'Engineer', 'Innovator / Founder'].map((persona, i) => (
          <div key={i} className={styles.card}>{persona}</div>
        ))}
      </div>
    </Section>
  );
  
  const WhyThisBook = () => (
    <Section title="Why This Book Is Different">
        <div className={styles.comparisonGrid}>
            <div className={styles.gridItem}>AI-first approach </div>
            <div className={styles.gridItem}>Real-world embodiment </div>
            <div className={styles.gridItem}>Simulation-to-real transfer </div>
            <div className={styles.gridItem}>Research-backed </div>
            <div className={styles.gridItem}>Industry-aligned </div>
        </div>
    </Section>
);
  
  const AuthorVision = () => (
    <Section title="Author / Project Vision">
      <p>"To build the next generation of embodied AI engineers for a humanoid future."</p>
      <p>Roadmap of future editions and open research contribution note to follow.</p>
    </Section>
  );
  
  const CallToActionFooter = () => (
    <footer className={styles.footer}>
      <h2>Enter the World of Physical Intelligence</h2>
      <div className={styles.heroButtons}>
        <a href="#" className={styles.buttonPrimary}>Start Reading</a>
        <a href="#" className={styles.buttonSecondary}>Join Research Community</a>
        <a href="https://github.com/MuhammadHassaanArain/PAI" className={styles.buttonSecondary}>GitHub Repository</a>
      </div>
      <div className={styles.newsletter}>
        <input type="email" placeholder="Enter your email" />
        <button>Subscribe</button>
      </div>
    </footer>
  );
  

export default function Home() {
  return (
    <Layout>
      <HeroSection />
      <main>
        <WhatIsPhysicalAI />
        <LearningOutcomes />
        <CurriculumRoadmap />
        <HandsOnPractice />
        <TargetLearners />
        <WhyThisBook />
        <AuthorVision />
      </main>
      <CallToActionFooter />
    </Layout>
  );
}