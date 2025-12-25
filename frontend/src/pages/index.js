import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const [animate, setAnimate] = useState(false);
  const [currentImageIndex, setCurrentImageIndex] = useState(0);

  // Topic images for the right box
  const topicImages = [
    {
      src: require('@site/static/img/humanoid-robot.png').default,
      alt: 'Humanoid Robot',
      title: 'Humanoid Robotics',
      subtitle: 'Advanced AI & Control Systems'
    },
    {
      src: require('@site/static/img/ros-2-logo.png').default,
      alt: 'ROS 2',
      title: 'ROS 2 Framework',
      subtitle: 'Robot Operating System 2'
    },
    {
      src: require('@site/static/img/isaac-sim.png').default,
      alt: 'Isaac Sim',
      title: 'Simulation',
      subtitle: 'NVIDIA Isaac Sim'
    },
    {
      src: require('@site/static/img/physical-ai.png').default,
      alt: 'Physical AI',
      title: 'Physical AI',
      subtitle: 'Embodied Intelligence'
    }
  ];

  useEffect(() => {
    // Trigger animations on mount
    setAnimate(true);

    // Rotate images every 5 seconds
    const interval = setInterval(() => {
      setCurrentImageIndex(prev => (prev + 1) % topicImages.length);
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  const currentImage = topicImages[currentImageIndex];

  return (
    <section className={styles.heroSection}>
      <div className={styles.heroContainer}>
        {/* LEFT CONTENT */}
        <div className={`${styles.heroContent} ${animate ? styles.slideInLeft : ''}`}>
          <h1 className={styles.heroTitle}>
            Physical AI & Humanoid Robotics
          </h1>
          <p className={styles.heroSubtitle}>
            A comprehensive guide to humanoid robotics with ROS 2, Simulation, and AI
          </p>
          <div className={styles.heroButtons}>
            <Link className={styles['button--primary']} to="/docs/intro">
              📚 Learn More
            </Link>
            <Link className={styles['button--secondary']} to="/docs/module-1-ros2/module-1-ros2-index">
              🚀 Start Learning
            </Link>
          </div>
        </div>

        {/* RIGHT BOX */}
        <div className={`${styles.heroImage} ${animate ? styles.slideInRight : ''}`}>
          <div className={styles.bookCover}>
            <img src={currentImage.src} alt={currentImage.alt} className={styles.topicImage} />
            <div className={styles.bookCoverTitle}>{currentImage.title}</div>
            <div className={styles.bookCoverSubtitle}>{currentImage.subtitle}</div>

            {/* Image indicators */}
            <div className={styles.imageIndicators}>
              {topicImages.map((_, index) => (
                <div
                  key={index}
                  className={`${styles.indicator} ${index === currentImageIndex ? styles.active : ''}`}
                  onClick={() => setCurrentImageIndex(index)}
                />
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  const [activeStep, setActiveStep] = useState(null);

  return (
    <Layout
      title={`Home - ${siteConfig.title}`}
      description="A comprehensive guide to humanoid robotics with ROS 2, Simulation, and AI">

      <HomepageHeader />

      <main>
        {/* FEATURES SECTION */}
        <section className={styles.featuresSection}>
          <div className={styles.featuresContainer}>
            <div className={styles.featuresGrid}>
              <div className={styles.featureCard} style={{animationDelay: '0s'}}>
                <div className={styles.featureIcon}>🤖</div>
                <h3 className={styles.featureTitle}>ROS 2 Fundamentals</h3>
                <p className={styles.featureDescription}>Master the Robot Operating System 2 with practical examples and real-world applications</p>

                <ul className={styles.featurePoints}>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Core ROS 2 concepts and architecture</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Node management & communication</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Topics, services, and actions</li>
                </ul>

                <div className={styles.noteBox}>
                  <p>ROS 2 provides a flexible framework for writing robot software with improved security and real-time support.</p>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.2s'}}>
                <div className={styles.keyConceptLabel}>Key Concept</div>
                <div className={styles.featureIcon}>🎮</div>
                <h3 className={styles.featureTitle}>Simulation Environments</h3>
                <p className={styles.featureDescription}>Learn physics simulation with Gazebo, Unity, and advanced visualization techniques</p>

                <ul className={styles.featurePoints}>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Gazebo physics engine integration</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Unity robotics simulation tools</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> NVIDIA Isaac Sim – Advanced Use</li>
                </ul>

                <div className={styles.noteBox}>
                  <p>Simulation environments allow safe testing of robot algorithms before deployment on real hardware.</p>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.4s'}}>
                <div className={styles.keyConceptLabel}>Key Concept</div>
                <div className={styles.featureIcon}>🧠</div>
                <h3 className={styles.featureTitle}>AI Integration</h3>
                <p className={styles.featureDescription}>Implement intelligent navigation and control systems using NVIDIA Isaac tools</p>

                <ul className={styles.featurePoints}>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Deep learning for robot perception</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Reinforcement learning applications</li>
                  <li className={styles.featurePoint}><span className={styles.pointBullet}>➤</span> Computer vision and sensor fusion</li>
                </ul>

                <div className={styles.noteBox}>
                  <p>AI integration enables robots to learn from experience and adapt to new environments autonomously.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

    {/* Interactive Book-Themed Step Boxes with Curved Animated Lines */}
    <section className={styles.stepBoxesSection}>
      <div className={styles.stepBoxesContainer}>
        <div className={styles.stepBoxWrapper}>
          <div
            className={`${styles.stepBox} ${activeStep === 'line1' ? styles.stepBoxActive : ''}`}
            id="step1"
            onMouseEnter={() => setActiveStep('line1')}
            onMouseLeave={() => setActiveStep(null)}
          >
            <div className={styles.stepNumber}>1</div>
            <h3 className={styles.stepTitle}>Core</h3>
            <p className={styles.stepDescription}>Concepts</p>
          </div>

          <div
            className={`${styles.stepBox} ${activeStep === 'line2' ? styles.stepBoxActive : ''}`}
            id="step2"
            onMouseEnter={() => setActiveStep('line2')}
            onMouseLeave={() => setActiveStep(null)}
          >
            <div className={styles.stepNumber}>2</div>
            <h3 className={styles.stepTitle}>Apply</h3>
            <p className={styles.stepDescription}>Practical</p>
          </div>

          <div
            className={`${styles.stepBox} ${activeStep === 'line3' ? styles.stepBoxActive : ''}`}
            id="step3"
            onMouseEnter={() => setActiveStep('line3')}
            onMouseLeave={() => setActiveStep(null)}
          >
            <div className={styles.stepNumber}>3</div>
            <h3 className={styles.stepTitle}>Advanced</h3>
            <p className={styles.stepDescription}>Topics</p>
          </div>


          <svg className={styles.stepLines} viewBox="0 0 1000 200" preserveAspectRatio="none">
            {/* Base lines - always visible with light color */}
            <path
              className={styles.stepLineBase}
              id="line1-base"
              d="M 200 90 Q 300 50 400 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="1.5"
              strokeOpacity="0.3"
            />
            <path
              className={styles.stepLineBase}
              id="line2-base"
              d="M 400 90 Q 500 130 600 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="1.5"
              strokeOpacity="0.3"
            />
            <path
              className={styles.stepLineBase}
              id="line3-base"
              d="M 600 90 Q 700 50 800 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="1.5"
              strokeOpacity="0.3"
            />

            {/* Animated lines - appear on hover */}
            <path
              className={`${styles.stepLine} ${activeStep === 'line1' ? styles.stepLineActive : ''}`}
              id="line1"
              d="M 200 90 Q 300 50 400 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="2.5"
              strokeOpacity="0"
            />
            <path
              className={`${styles.stepLine} ${activeStep === 'line2' ? styles.stepLineActive : ''}`}
              id="line2"
              d="M 400 90 Q 500 130 600 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="2.5"
              strokeOpacity="0"
            />
            <path
              className={`${styles.stepLine} ${activeStep === 'line3' ? styles.stepLineActive : ''}`}
              id="line3"
              d="M 600 90 Q 700 50 800 90"
              fill="none"
              stroke="#0f3460"
              strokeWidth="2.5"
              strokeOpacity="0"
            />
          </svg>
        </div>
      </div>
    </section>

        {/* BOOK MODULES & CHAPTERS */}
        <section className={styles.section}>
          <div className={styles.sectionContainer}>
            <h2 className={styles.sectionTitle}>Book Modules & Chapters</h2>
            <p className={styles.sectionSubtitle}>
              Explore comprehensive modules covering all aspects of humanoid robotics from fundamentals to advanced implementations.
            </p>
            <div className={styles.featuresGrid}>
              <div className={styles.featureCard} style={{animationDelay: '0.1s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    1
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      85% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>ROS 2 Framework</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Master the Robot Operating System 2 with practical examples</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Learn core concepts, node management, and communication</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Foundation for robotics</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/module-1-ros2/module-1-ros2-index">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.2s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    2
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      70% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>Simulation Environments</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Physics simulation with Gazebo, Unity, and advanced visualization</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Safe testing of robot algorithms before deployment on real hardware</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Virtual testing for deployment</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/module-2-simulation/">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.3s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    3
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      60% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>AI Integration</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Implement intelligent navigation and control systems using NVIDIA Isaac tools</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Deep learning, reinforcement learning, and computer vision applications</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Intelligent systems for behavior</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/module-3-ai-integration/">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.4s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    4
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      45% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>Hardware Integration</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Connect software with physical hardware for complete robot functionality</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Sensor fusion, actuator control, and real-time systems</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Physical systems for interaction</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/hardware-requirements/">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.5s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    5
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      30% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>Safety & Ethics</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Essential guidelines for responsible humanoid robotics development</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Safety protocols, ethics, and regulatory compliance</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Responsible development practices</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/safety-ethical-guidelines/">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
              <div className={styles.featureCard} style={{animationDelay: '0.6s'}}>
                <div style={{textAlign: 'center'}}>
                  <div style={{
                    width: '50px',
                    height: '50px',
                    borderRadius: '50%',
                    backgroundColor: '#0f3460',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '1.2rem',
                    margin: '0 auto 1rem auto'
                  }}>
                    6
                  </div>
                  <div style={{display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem', justifyContent: 'center'}}>
                    <div style={{
                      backgroundColor: '#e2e8f0',
                      padding: '0.25rem 0.5rem',
                      borderRadius: '0.5rem',
                      fontSize: '0.8rem',
                      fontWeight: '600',
                      color: '#2d3748'
                    }}>
                      20% Complete
                    </div>
                  </div>
                  <h3 className={styles.featureTitle}>Advanced Applications</h3>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Explore the frontier of humanoid robotics research and applications</p>
                  <p className={styles.featureDescription} style={{textAlign: 'left', marginLeft: '0', fontSize: '0.9rem'}}>Research methodologies, emerging technologies, and future trends</p>

                  <div className={styles.noteBox} style={{backgroundColor: '#e2e8f0', padding: '0.5rem', borderRadius: '0.5rem'}}>
                    <p style={{margin: 0, fontSize: '0.85rem', color: '#2d3748'}}>Research and innovation in robotics</p>
                  </div>

                  <div style={{marginTop: '1rem', textAlign: 'left'}}>
                    <Link className={styles['button--secondary']} style={{fontSize: '0.8rem', padding: '0.25rem 0.5rem'}} to="/docs/capstone/">
                      Read Book
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* GET STARTED */}
        <section className={styles.section}>
          <div className={styles.sectionContainer}>
            <h2 className={styles.sectionTitle}>Ready to Start Your Journey?</h2>
            <p className={styles.sectionSubtitle}>
              Join thousands of students and professionals learning the future of robotics
            </p>
            <div style={{display: 'flex', justifyContent: 'center', gap: '1rem', flexWrap: 'wrap'}}>
              <Link className={styles['button--primary']} to="/docs/intro">
                🌟 Get Started Now
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}