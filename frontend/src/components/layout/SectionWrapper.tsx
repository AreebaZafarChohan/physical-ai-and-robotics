import React from 'react';

interface SectionWrapperProps {
  children: React.ReactNode;
  id?: string;
  className?: string;
  background?: 'default' | 'dark' | 'light' | 'gradient' | 'neon';
  padding?: 'sm' | 'md' | 'lg' | 'xl';
  centered?: boolean;
  containerWidth?: 'sm' | 'md' | 'lg' | 'xl' | 'full';
}

const SectionWrapper: React.FC<SectionWrapperProps> = ({
  children,
  id,
  className = '',
  background = 'default',
  padding = 'md',
  centered = false,
  containerWidth = 'lg',
}) => {
  const backgroundClasses = {
    default: 'bg-background',
    dark: 'bg-dark-bg',
    light: 'bg-light-bg',
    gradient: 'bg-gradient-to-b from-dark-bg to-dark-card',
    neon: 'bg-gradient-neon-combo',
  };

  const paddingClasses = {
    sm: 'py-8',
    md: 'py-16',
    lg: 'py-24',
    xl: 'py-32',
  };

  const containerWidthClasses = {
    sm: 'max-w-screen-sm',
    md: 'max-w-screen-md',
    lg: 'max-w-screen-lg',
    xl: 'max-w-screen-xl',
    full: 'max-w-full',
  };

  const centerClass = centered ? 'flex flex-col items-center justify-center' : '';
  
  return (
    <section 
      id={id}
      className={`${backgroundClasses[background]} ${paddingClasses[padding]} ${centerClass} ${className}`}
    >
      <div className={`w-full ${containerWidthClasses[containerWidth]} mx-auto px-4 md:px-8`}>
        {children}
      </div>
    </section>
  );
};

export default SectionWrapper;