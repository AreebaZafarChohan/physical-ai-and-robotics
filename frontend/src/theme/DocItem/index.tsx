import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import PersonalizeContentButton from '../../components/PersonalizeContentButton';

export default function DocItemWrapper(props: any) {
  const [currentPath, setCurrentPath] = useState('');

  useEffect(() => {
    if (typeof window !== 'undefined') {
      setCurrentPath(window.location.pathname);
    }
  }, []);

  return (
    <>
      {/* Inject button BEFORE actual doc - NO top margin */}
      <div className="container margin-bottom--md">
        <div className="flex justify-end">
          <PersonalizeContentButton
            chapterPath={currentPath}
            onPersonalize={() => {}}
          />
        </div>
      </div>

      {/* ORIGINAL DOC ITEM */}
      <DocItem {...props} />
    </>
  );
}