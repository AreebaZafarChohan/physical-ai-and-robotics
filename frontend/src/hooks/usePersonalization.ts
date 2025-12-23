// frontend/src/hooks/usePersonalization.ts
import { useState, useEffect } from 'react';
import { fetchPersonalizationData } from '../services/user_profile';

interface PersonalizationData {
  software_background: string[];
  hardware_background: string[];
}

interface UsePersonalizationResult {
  personalizationData: PersonalizationData | null;
  isLoading: boolean;
  error: string | null;
}

export const usePersonalization = (): UsePersonalizationResult => {
  const [personalizationData, setPersonalizationData] = useState<PersonalizationData | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const getPersonalizationData = async () => {
      try {
        setIsLoading(true);
        const data = await fetchPersonalizationData();
        setPersonalizationData(data);
      } catch (err: any) {
        setError(err.message || 'Failed to fetch personalization data.');
      } finally {
        setIsLoading(false);
      }
    };

    getPersonalizationData();
  }, []);

  return { personalizationData, isLoading, error };
};
