import React from 'react';
import { motion } from 'framer-motion';
import SectionWrapper from '../layout/SectionWrapper';
import SpectrumCard from '../core/SpectrumCard';

const AISpectrumSection: React.FC = () => {
  const spectrumItems = [
    {
      type: 'assisted' as const,
      title: 'AI Assisted',
      description: 'Human-guided systems where AI enhances human capabilities and decision-making processes.',
      icon: '🦾',
    },
    {
      type: 'driven' as const,
      title: 'AI Driven',
      description: 'Semi-autonomous systems that operate with minimal human intervention in specific domains.',
      icon: '🤖',
    },
    {
      type: 'native' as const,
      title: 'AI Native',
      description: 'Fully autonomous systems that function independently with sophisticated decision-making capabilities.',
      icon: '🚀',
    },
  ];

  return (
    <SectionWrapper background="dark" padding="lg">
      <div className="text-center mb-16">
        <motion.h2 
          className="text-3xl md:text-4xl font-bold text-white mb-4"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
        >
          Understanding the <span className="text-neon-purple-400">AI Spectrum</span>
        </motion.h2>
        <motion.p 
          className="text-xl text-white/80 max-w-2xl mx-auto"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.1 }}
        >
          Explore how artificial intelligence is integrated with physical systems across different levels of autonomy
        </motion.p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
        {spectrumItems.map((item, index) => (
          <motion.div
            key={index}
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5, delay: index * 0.1 }}
          >
            <SpectrumCard
              type={item.type}
              title={item.title}
              description={item.description}
              icon={<span className="text-3xl">{item.icon}</span>}
            />
          </motion.div>
        ))}
      </div>
    </SectionWrapper>
  );
};

export default AISpectrumSection;