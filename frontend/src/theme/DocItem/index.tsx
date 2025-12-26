import React, {useState, useEffect} from 'react';
import DocItem from '@theme-original/DocItem';
import type {WrapperProps} from '@docusaurus/types';
import type DocItemType from '@theme/DocItem';
import PersonalizeContentButton from '../../components/PersonalizeContentButton';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props) {
  const [currentPath, setCurrentPath] = useState('');

  useEffect(() => {
    if (typeof window !== 'undefined') {
      setCurrentPath(window.location.pathname);
    }
  }, []);

  return (
    <>
      {/* Inject button BEFORE actual doc */}
      <div className="container margin-top--md margin-bottom--md">
        <div className="flex justify-end">
          <PersonalizeContentButton
            chapterPath={currentPath}
            onPersonalize={() => {}}
          />
        </div>
      </div>

      {/* ORIGINAL DOC ITEM — VERY IMPORTANT */}
      <DocItem {...props} />
    </>
  );
}
