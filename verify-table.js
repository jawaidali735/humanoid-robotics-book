import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import dotenv from 'dotenv';

dotenv.config();

// Use the DATABASE_URL from the root .env file
console.log('Using DATABASE_URL:', process.env.DATABASE_URL);

// Create the database connection
const client = neon(process.env.DATABASE_URL);
const db = drizzle(client);

async function verifyTable() {
  try {
    console.log('Verifying users table exists...');

    // Query to check if the users table exists
    const result = await db.execute(`
      SELECT EXISTS (
        SELECT FROM information_schema.tables
        WHERE table_schema = 'public'
        AND table_name = 'users'
      ) as table_exists;
    `);

    console.log('Table exists:', result.rows[0].table_exists);

    if (result.rows[0].table_exists) {
      // Check the table structure
      const structure = await db.execute(`
        SELECT column_name, data_type, is_nullable, column_default
        FROM information_schema.columns
        WHERE table_name = 'users'
        ORDER BY ordinal_position;
      `);

      console.log('Users table structure:');
      console.table(structure.rows);

      // Check if there are any users in the table
      const userCount = await db.execute(`
        SELECT COUNT(*) as count FROM users;
      `);

      console.log('Number of users in the table:', userCount.rows[0].count);
    } else {
      console.log('Users table does not exist!');

      // Let's check what tables exist
      const tables = await db.execute(`
        SELECT table_name
        FROM information_schema.tables
        WHERE table_schema = 'public'
        ORDER BY table_name;
      `);

      console.log('Available tables in the database:');
      console.table(tables.rows);
    }

  } catch (error) {
    console.error('Error verifying table:', error);
  } finally {
    // Note: neon client for http doesn't have a destroy method
    console.log('Verification completed.');
  }
}

verifyTable();