import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import dotenv from 'dotenv';

dotenv.config();

// Use the DATABASE_URL from the root .env file
console.log('Creating users table in database:', process.env.DATABASE_URL);

// Create the database connection
const client = neon(process.env.DATABASE_URL);
const db = drizzle(client);

async function createUsersTable() {
  try {
    console.log('Creating users table...');

    // Create the users table with proper SQL
    await db.execute(`
      CREATE TABLE IF NOT EXISTS users (
        id SERIAL PRIMARY KEY,
        email VARCHAR(255) NOT NULL UNIQUE,
        name VARCHAR(255),
        avatar VARCHAR(500),
        google_id VARCHAR(255) UNIQUE,
        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
      );
    `);

    console.log('Users table created successfully!');

    // Verify the table exists
    const result = await db.execute(`
      SELECT EXISTS (
        SELECT FROM information_schema.tables
        WHERE table_schema = 'public'
        AND table_name = 'users'
      ) as table_exists;
    `);

    console.log('Table exists after creation:', result.rows[0].table_exists);

    if (result.rows[0].table_exists) {
      // Show the table structure
      const structure = await db.execute(`
        SELECT column_name, data_type, is_nullable
        FROM information_schema.columns
        WHERE table_name = 'users'
        ORDER BY ordinal_position;
      `);

      console.log('Users table structure:');
      console.table(structure.rows);
    }

  } catch (error) {
    console.error('Error creating table:', error);
  } finally {
    console.log('Table creation process completed.');
  }
}

createUsersTable();