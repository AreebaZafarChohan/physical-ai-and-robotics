<<<<<<< HEAD
import asyncpg
import os
import logging
from dotenv import load_dotenv

load_dotenv()

logger = logging.getLogger(__name__)

DATABASE_URL = os.getenv("NEON_DATABASE_URL")

async def get_connection():
    if not DATABASE_URL:
        logger.error("NEON_DATABASE_URL environment variable not set.")
        raise ValueError("NEON_DATABASE_URL environment variable not set.")
    
    # Fix for incorrect DSN scheme
    fixed_db_url = DATABASE_URL.replace("postgresql+asyncpg://", "postgresql://")

    conn = None
    try:
        conn = await asyncpg.connect(fixed_db_url)
        return conn
    except Exception as e:
        logger.error(f"Failed to connect to the database: {e}")
        if conn:
            await conn.close()
        raise

async def create_tables():
    conn = None
    try:
        conn = await get_connection()
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id SERIAL PRIMARY KEY,
                username VARCHAR(50) UNIQUE NOT NULL,
                email VARCHAR(100) UNIQUE NOT NULL,
                hashed_password VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        ''')
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS user_memory (
                id SERIAL PRIMARY KEY,
                user_id INTEGER NOT NULL REFERENCES users(id),
                key VARCHAR(50) NOT NULL,
                value TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        ''')
        logger.info("Database tables 'users' and 'user_memory' checked/created successfully.")
    except Exception as e:
        logger.error(f"Error creating tables: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def fetch_user_by_username(username: str):
    conn = None
    try:
        conn = await get_connection()
        user_record = await conn.fetchrow("SELECT id, username, email, hashed_password FROM users WHERE username = $1", username)
        return user_record
    except Exception as e:
        logger.error(f"Error fetching user by username: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def create_new_user(username: str, email: str, hashed_password: str):
    conn = None
    try:
        conn = await get_connection()
        user_id = await conn.fetchval(
            "INSERT INTO users (username, email, hashed_password) VALUES ($1, $2, $3) RETURNING id",
            username, email, hashed_password
        )
        return user_id
    except Exception as e:
        logger.error(f"Error creating new user: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def save_user_memory(user_id: int, key: str, value: str):
    conn = None
    try:
        conn = await get_connection()
        await conn.execute(
            "INSERT INTO user_memory (user_id, key, value) VALUES ($1, $2, $3)",
            user_id, key, value
        )
        logger.info(f"Saved memory for user {user_id}: {key}={value}")
    except Exception as e:
        logger.error(f"Error saving user memory: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def get_user_memory(user_id: int):
    conn = None
    try:
        conn = await get_connection()
        memory_records = await conn.fetch("SELECT key, value FROM user_memory WHERE user_id = $1", user_id)
        return {record['key']: record['value'] for record in memory_records}
    except Exception as e:
        logger.error(f"Error retrieving user memory: {e}")
        raise
    finally:
        if conn:
            await conn.close()
=======
import asyncpg
import os
import logging
from dotenv import load_dotenv

load_dotenv()

logger = logging.getLogger(__name__)

DATABASE_URL = os.getenv("NEON_DATABASE_URL")

async def get_connection():
    if not DATABASE_URL:
        logger.error("NEON_DATABASE_URL environment variable not set.")
        raise ValueError("NEON_DATABASE_URL environment variable not set.")
    
    # Fix for incorrect DSN scheme
    fixed_db_url = DATABASE_URL.replace("postgresql+asyncpg://", "postgresql://")

    conn = None
    try:
        conn = await asyncpg.connect(fixed_db_url)
        return conn
    except Exception as e:
        logger.error(f"Failed to connect to the database: {e}")
        if conn:
            await conn.close()
        raise

async def create_tables():
    conn = None
    try:
        conn = await get_connection()
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id SERIAL PRIMARY KEY,
                username VARCHAR(50) UNIQUE NOT NULL,
                email VARCHAR(100) UNIQUE NOT NULL,
                hashed_password VARCHAR(255) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        ''')
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS user_memory (
                id SERIAL PRIMARY KEY,
                user_id INTEGER NOT NULL REFERENCES users(id),
                key VARCHAR(50) NOT NULL,
                value TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        ''')
        logger.info("Database tables 'users' and 'user_memory' checked/created successfully.")
    except Exception as e:
        logger.error(f"Error creating tables: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def fetch_user_by_username(username: str):
    conn = None
    try:
        conn = await get_connection()
        user_record = await conn.fetchrow("SELECT id, username, email, hashed_password FROM users WHERE username = $1", username)
        return user_record
    except Exception as e:
        logger.error(f"Error fetching user by username: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def create_new_user(username: str, email: str, hashed_password: str):
    conn = None
    try:
        conn = await get_connection()
        user_id = await conn.fetchval(
            "INSERT INTO users (username, email, hashed_password) VALUES ($1, $2, $3) RETURNING id",
            username, email, hashed_password
        )
        return user_id
    except Exception as e:
        logger.error(f"Error creating new user: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def save_user_memory(user_id: int, key: str, value: str):
    conn = None
    try:
        conn = await get_connection()
        await conn.execute(
            "INSERT INTO user_memory (user_id, key, value) VALUES ($1, $2, $3)",
            user_id, key, value
        )
        logger.info(f"Saved memory for user {user_id}: {key}={value}")
    except Exception as e:
        logger.error(f"Error saving user memory: {e}")
        raise
    finally:
        if conn:
            await conn.close()

async def get_user_memory(user_id: int):
    conn = None
    try:
        conn = await get_connection()
        memory_records = await conn.fetch("SELECT key, value FROM user_memory WHERE user_id = $1", user_id)
        return {record['key']: record['value'] for record in memory_records}
    except Exception as e:
        logger.error(f"Error retrieving user memory: {e}")
        raise
    finally:
        if conn:
            await conn.close()
>>>>>>> 012-docusaurus-i18n-urdu
