import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Head from '@docusaurus/Head';

import { useHistory } from '@docusaurus/router'; // Import useHistory for navigation


import HeroSection from '../components/sections/HeroSection';
import AISpectrumSection from '../components/sections/AISpectrumSection';
import FeatureGridSection from '../components/sections/FeatureGridSection';
import CTAFooterSection from '../components/sections/CTAFooterSection';


const Home: React.FC = () => {

  const history = useHistory();

  return (
    <Layout
      title="RoboX - Physical AI & Humanoid Robotics Platform"
      description="Explore the future of artificial intelligence embodied in physical form."
    >
      <Head>
        <meta
          name="description"
          content="Explore the future of artificial intelligence embodied in physical form. Learn how humanoid robots are reshaping our world."
        />
      </Head>

      <main className="bg-dark-bg text-white">

        <HeroSection />
        <AISpectrumSection />
        <FeatureGridSection />
        <CTAFooterSection />
      </main>
    </Layout>
  );
};

export default Home;
