import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <section className={styles.heroSection}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className={styles.heroSubtitle}>
            A comprehensive guide to humanoid robotics with ROS 2, Simulation, and AI
          </p>
          <div className={styles.heroButtons}>
            <Link
              className={styles['button--primary']}
              to="/docs/intro">
              📚 Learn More
            </Link>
            <Link
              className={styles['button--secondary']}
              to="/docs/module-1-ros2/">
              🚀 Start Learning
            </Link>
          </div>
        </div>
        <div className={styles.heroImage}>
          <div className={styles.bookCover}>
            <div className={styles.bookCoverTitle}>Humanoid Robotics</div>
            <div className={styles.bookCoverSubtitle}>Mastering ROS 2, Simulation & AI</div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="A comprehensive guide to humanoid robotics with ROS 2, Simulation, and AI">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className={styles.featuresContainer}>
            <div className={styles.featuresGrid}>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🤖</div>
                <h3 className={styles.featureTitle}>ROS 2 Fundamentals</h3>
                <p className={styles.featureDescription}>Master the Robot Operating System 2 with practical examples and real-world applications</p>
              </div>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🎮</div>
                <h3 className={styles.featureTitle}>Simulation Environments</h3>
                <p className={styles.featureDescription}>Learn physics simulation with Gazebo, Unity, and advanced visualization techniques</p>
              </div>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🧠</div>
                <h3 className={styles.featureTitle}>AI Integration</h3>
                <p className={styles.featureDescription}>Implement intelligent navigation and control systems using NVIDIA Isaac tools</p>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.section}>
          <div className={styles.sectionContainer}>
            <h2 className={styles.sectionTitle}>Perfect for Everyone</h2>
            <p className={styles.sectionSubtitle}>
              Whether you're a student, researcher, or professional, this comprehensive guide provides everything you need to build advanced humanoid robotic systems from scratch.
            </p>
            <div className={styles.featuresGrid}>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🎯</div>
                <h3 className={styles.featureTitle}>Step-by-step Tutorials</h3>
                <p className={styles.featureDescription}>Practical examples with code samples</p>
              </div>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>📚</div>
                <h3 className={styles.featureTitle}>Real-world Case Studies</h3>
                <p className={styles.featureDescription}>Applications and implementation examples</p>
              </div>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🛠️</div>
                <h3 className={styles.featureTitle}>Hands-on Projects</h3>
                <p className={styles.featureDescription}>Exercises and debugging best practices</p>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.section}>
          <div className={styles.sectionContainer}>
            <h2 className={styles.sectionTitle}>Ready to Start Your Journey?</h2>
            <p className={styles.sectionSubtitle}>
              Join thousands of students and professionals learning the future of robotics
            </p>
            <div style={{display: 'flex', justifyContent: 'center', gap: '1rem', flexWrap: 'wrap'}}>
              <Link
                className={styles['button--primary']}
                to="/docs/intro">
                🌟 Get Started Now
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}