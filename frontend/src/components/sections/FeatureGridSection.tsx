import React from 'react';
import { motion } from 'framer-motion';
import SectionWrapper from '../layout/SectionWrapper';
import FeatureCard from '../core/FeatureCard';
import GridWrapper from '../layout/GridWrapper';

const FeatureGridSection: React.FC = () => {
  const features = [
    {
      title: 'Embodied Cognition',
      description: 'Understanding how physical form influences cognitive processes in AI systems.',
      icon: '🧠',
      iconColor: 'neon-purple' as const,
      badge: 'Core Concept',
    },
    {
      title: 'Locomotion Systems',
      description: 'Advanced walking, crawling, and movement algorithms for humanoid robots.',
      icon: '🦵',
      iconColor: 'neon-pink' as const,
      badge: 'Technical',
    },
    {
      title: 'Perception & Sensing',
      description: 'Multi-modal sensory systems for environmental awareness and interaction.',
      icon: '👁️',
      iconColor: 'primary' as const,
      badge: 'Hardware',
    },
    {
      title: 'Human-Robot Interaction',
      description: 'Natural interfaces for seamless collaboration between humans and robots.',
      icon: '🤝',
      iconColor: 'secondary' as const,
      badge: 'UX',
    },
    {
      title: 'Learning & Adaptation',
      description: 'Machine learning techniques that allow robots to improve with experience.',
      icon: '📚',
      iconColor: 'accent' as const,
      badge: 'AI',
    },
    {
      title: 'Safety Protocols',
      description: 'Built-in safety measures to ensure responsible deployment of robotic systems.',
      icon: '🛡️',
      iconColor: 'primary' as const,
      badge: 'Safety',
    },
    {
      title: 'Ethical Frameworks',
      description: 'Guidelines for the responsible development and use of physical AI systems.',
      icon: '⚖️',
      iconColor: 'neon-purple' as const,
      badge: 'Ethics',
    },
    {
      title: 'Open Research',
      description: 'Open source tools and datasets to accelerate robotics research.',
      icon: '🔬',
      iconColor: 'neon-pink' as const,
      badge: 'Community',
    },
  ];

  return (
    <SectionWrapper background="dark" padding="lg">
      <div className="text-center mb-16">
        <motion.h2 
          className="text-3xl md:text-4xl font-bold text-light-text mb-4"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
        >
          Key <span className="text-neon-purple-400">Concepts</span> in Physical AI
        </motion.h2>
        <motion.p 
          className="text-xl text-light-text/80 max-w-2xl mx-auto"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.1 }}
        >
          Explore the fundamental principles and technologies that drive humanoid robotics
        </motion.p>
      </div>

      <GridWrapper cols={4} gap="lg" responsive={true}>
        {features.map((feature, index) => (
          <motion.div
            key={index}
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: index * 0.1 }}
          >
            <FeatureCard
              title={feature.title}
              description={feature.description}
              icon={<span className="text-2xl">{feature.icon}</span>}
              iconColor={feature.iconColor}
              badge={feature.badge}
              animated={true}
            />
          </motion.div>
        ))}
      </GridWrapper>
    </SectionWrapper>
  );
};

export default FeatureGridSection;