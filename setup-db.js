import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import { pgTable, serial, varchar, timestamp } from 'drizzle-orm/pg-core';
import dotenv from 'dotenv';

dotenv.config();

// Define the same schema as in the auth server
const users = pgTable('users', {
  id: serial('id').primaryKey(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  name: varchar('name', { length: 255 }),
  avatar: varchar('avatar', { length: 500 }),
  googleId: varchar('google_id', { length: 255 }).unique(), // For Google OAuth
  createdAt: timestamp('created_at').defaultNow().notNull(),
  updatedAt: timestamp('updated_at').defaultNow().notNull(),
});

// Create the database connection
const client = neon(process.env.DATABASE_URL);
const db = drizzle(client, { schema: { users } });

async function setupDatabase() {
  try {
    console.log('Setting up database schema...');

    // Create the users table if it doesn't exist
    await client.execute(`
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

    console.log('Database schema setup completed successfully!');

    // Verify the table exists by doing a simple query
    const result = await client.execute('SELECT column_name, data_type FROM information_schema.columns WHERE table_name = \'users\'');
    console.log('Users table columns:', result.rows);

  } catch (error) {
    console.error('Error setting up database:', error);
  } finally {
    // Close the connection
    client.destroy();
  }
}

setupDatabase();