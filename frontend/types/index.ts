export interface UserCreate {
    email: string;
    name: string;
    password?: string;
    software_level?: string;
    hardware_level?: string;
    programming_languages?: string[];
    robotics_experience?: boolean;
    ai_experience?: boolean;
}

export interface UserLogin {
    email: string;
    password?: string;
}

export interface UserProfile {
    id: string;
    email: string;
    name: string;
    auth_provider: string;
    background?: {
        software_level?: string;
        hardware_level?: string;
        programming_languages?: string[];
        robotics_experience?: boolean;
        ai_experience?: boolean;
    }
}
