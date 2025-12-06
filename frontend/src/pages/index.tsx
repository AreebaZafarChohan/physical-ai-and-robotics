import React from 'react';
import Head from '@docusaurus/Head';
import Navbar from '../components/core/Navbar';
import Footer from '../components/core/Footer';
import HeroSection from '../components/sections/HeroSection';
import AISpectrumSection from '../components/sections/AISpectrumSection';
import FeatureGridSection from '../components/sections/FeatureGridSection';
import CTAFooterSection from '../components/sections/CTAFooterSection';

const Home: React.FC = () => {
  return (
    <div className="min-h-screen bg-dark-bg text-light-text">
      <Head>
        <title>Physical AI & Humanoid Robotics Platform</title>
        <meta name="description" content="Explore the future of artificial intelligence embodied in physical form. Learn how humanoid robots are reshaping our world through cutting-edge research and applications." />
      </Head>

      <Navbar />
      
      <main>
        <HeroSection />
        <AISpectrumSection />
        <FeatureGridSection />
        <CTAFooterSection />
      </main>
      
      <Footer />
    </div>
  );
};

export default Home;