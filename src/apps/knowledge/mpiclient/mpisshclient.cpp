#include "mpisshclient.h"

#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include "common/app_cppcommon.h"
#include "utils/Paths.h"

namespace mpiclient {

    MPISSHClient::MPISSHClient() {
        connect();
    }
    
    MPISSHClient::~MPISSHClient() {
    }

    bool MPISSHClient::submitProblem() {
        LOG(INFO) << FUNCTION_NAME();
        bool ret =  send(RTQL8_DATA_PATH"xml/test.xml", "/nv/hp16/sha9/knowledge-rtql8/data/xml/test.xml");
        LOG(INFO) << FUNCTION_NAME() << " : OK";
        return true;
    }
    
    bool MPISSHClient::executeSolver() {
        LOG(INFO) << FUNCTION_NAME();
        bool ret = execute("python /nv/hp16/sha9/optimize.py");
        LOG(INFO) << "Optimiation must be submitted, I guess";
        while(1) {
            std::string ret = read("/nv/hp16/sha9/status.txt");
            LOG(INFO) << "status = " << ret;
            if (ret.find("done") != std::string::npos) {
                LOG(INFO) << "Wow! optimiation seems done!";
                break;
            }
            sleep(3);
        }
        LOG(INFO) << FUNCTION_NAME() << " : OK";
        return ret;
    }
    
    bool MPISSHClient::receiveStatus() {
        return true;
    }
    
    bool MPISSHClient::receiveResult() {
        LOG(INFO) << FUNCTION_NAME();
        bool ret =  recv(RTQL8_DATA_PATH"xml/result.xml", "/nv/hp16/sha9/knowledge-rtql8/data/xml/result.xml");
        LOG(INFO) << FUNCTION_NAME() << " : OK";
        return true;
    }
        

    bool MPISSHClient::send(const char* const local, const char* remote) {
        using namespace std;
        ifstream fin(local, ios::binary);

        fin.seekg(0, ios::end);
        ios::pos_type bufsize = fin.tellg(); // get file size in bytes
        fin.seekg(0); // rewind to beginning of file

        char* buf = new char[bufsize];
        fin.read(buf, bufsize); // read file contents into buffer

        int rc;
        int access_type = O_WRONLY | O_CREAT | O_TRUNC;
        sftp_file file;
        file = sftp_open(sftp, remote, access_type, S_IRWXU);
        if (file == NULL)
        {
            fprintf(stderr, "Can't open file for writing: %s\n",
                    ssh_get_error(ssh));
            return false;
        }        
        sftp_write(file, buf, bufsize); // write to remote file

        delete[] buf;

        rc = sftp_close(file);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Can't close the written file: %s\n",
                    ssh_get_error(ssh));
            return false;
        }

        return true;

    }

    std::string MPISSHClient::read(const char* remote) {
        int access_type;
        sftp_file file;
        char buffer[1024];
        int nbytes, rc;
        std::string ret = "";
        access_type = O_RDONLY;
        file = sftp_open(sftp, remote,
                         access_type, 0);
        if (file == NULL)
        {
            // fprintf(stderr, "Can't open file for reading: %s\n",
            //         ssh_get_error(ssh));
            return ret;
        }
        nbytes = sftp_read(file, buffer, sizeof(buffer));
        // if (nbytes > 0) {
        //     ret += buffer;
        //     strcpy(buffer, "");
        // }
        while (nbytes > 0)
        {
            if (write(1, buffer, nbytes) != nbytes)
            {
                sftp_close(file);
                return ret;
            }
            if (nbytes > 0) {
                ret += buffer;
                strcpy(buffer, "");
            }
            nbytes = sftp_read(file, buffer, sizeof(buffer));
        }
        if (nbytes < 0)
        {
            fprintf(stderr, "Error while reading file: %s\n",
                    ssh_get_error(ssh));
            sftp_close(file);
            return ret;
        }
        rc = sftp_close(file);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Can't close the read file: %s\n",
                    ssh_get_error(ssh));
            return ret;
        }
        return ret;
    }
    
    
    bool MPISSHClient::recv(const char* const local, const char* remote) {
        int access_type;
        sftp_file file;
        char buffer[1024];
        int nbytes, rc;
        int filewrite = open(local,O_WRONLY);
        access_type = O_RDONLY;
        file = sftp_open(sftp, remote,
                         access_type, 0);
        if (file == NULL)
        {
            // fprintf(stderr, "Can't open file for reading: %s\n",
            //         ssh_get_error(ssh));
            return false;
        }
        nbytes = sftp_read(file, buffer, sizeof(buffer));
        // if (nbytes > 0) {
        //     ret += buffer;
        //     strcpy(buffer, "");
        // }
        while (nbytes > 0)
        {
            // if (write(1, buffer, nbytes) != nbytes)
            if (write(filewrite, buffer, nbytes) != nbytes)
            {
                sftp_close(file);
                return false;
            }
            nbytes = sftp_read(file, buffer, sizeof(buffer));
        }
        if (nbytes < 0)
        {
            fprintf(stderr, "Error while reading file: %s\n",
                    ssh_get_error(ssh));
            sftp_close(file);
            return false;
        }
        rc = sftp_close(file);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Can't close the read file: %s\n",
                    ssh_get_error(ssh));
            return false;
        }
        return true;
    }
    
    bool MPISSHClient::execute(const char* const cmd) {
        ssh_channel channel;
        int rc;
        char buffer[256];
        unsigned int nbytes;

        channel = ssh_channel_new(ssh);
        if (channel == NULL)
            return SSH_ERROR;
        rc = ssh_channel_open_session(channel);
        if (rc != SSH_OK)
        {
            ssh_channel_free(channel);
            return false;
        }
        // rc = ssh_channel_request_exec(channel, "python /nv/hp16/sha9/optimize.py");
        rc = ssh_channel_request_exec(channel, cmd);
        if (rc != SSH_OK)
        {
            ssh_channel_close(channel);
            ssh_channel_free(channel);
            return false;
        }
        nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
        while (nbytes > 0)
        {
            if (write(1, buffer, nbytes) != nbytes)
            {
                ssh_channel_close(channel);
                ssh_channel_free(channel);
                return false;
            }
            nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
        }
    
        if (nbytes < 0)
        {
            ssh_channel_close(channel);
            ssh_channel_free(channel);
            return false;
        }
        ssh_channel_send_eof(channel);
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        return true;
    }

    void MPISSHClient::connect() {
        int rc;
        // char *password;
        char password[100] = "gahee!4101910";
        // Open session and set options
        ssh = ssh_new();
        if (ssh == NULL) {
            LOG(FATAL) << "cannot connect to SSH server";
            return;
        }
        
        ssh_options_set(ssh, SSH_OPTIONS_HOST, "force-6.pace.gatech.edu");
        ssh_options_set(ssh, SSH_OPTIONS_USER, "sha9");
    
        // Connect to server
        rc = ssh_connect(ssh);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Error connecting to localhost: %s\n",
                    ssh_get_error(ssh));
            ssh_free(ssh);
            exit(-1);
        }

        LOG(INFO) << FUNCTION_NAME() << " : connect OK";

        // Verify the server's identity
        // For the source code of verify_knowhost(), check previous example
        if (verify_knownhost(ssh) < 0)
        {
            ssh_disconnect(ssh);
            ssh_free(ssh);
            exit(-1);
        }

        LOG(INFO) << FUNCTION_NAME() << " : verification OK";


        // Authenticate ourselves
        // password = getpass("Password: ");
        rc = ssh_userauth_password(ssh, NULL, password);
        if (rc != SSH_AUTH_SUCCESS)
        {
            fprintf(stderr, "Error authenticating with password: %s\n",
                    ssh_get_error(ssh));
            ssh_disconnect(ssh);
            ssh_free(ssh);
            exit(-1);
        }
  

        LOG(INFO) << FUNCTION_NAME() << " : authentication OK";

        sftp = sftp_new(ssh);
        if (sftp == NULL)
        {
            fprintf(stderr, "Error allocating SFTP session: %s\n",
                    ssh_get_error(ssh));
            exit(-1);

        }
        LOG(INFO) << FUNCTION_NAME() << " : SFTP allocating OK";
        rc = sftp_init(sftp);
        if (rc != SSH_OK)
        {
            fprintf(stderr, "Error initializing SFTP session: %d.\n",
                    sftp_get_error(sftp));
            sftp_free(sftp);
            exit(-1);
        }
        LOG(INFO) << FUNCTION_NAME() << " : SFTP initializing OK";

        LOG(INFO) << FUNCTION_NAME() << " OK";
    }

    int MPISSHClient::verify_knownhost(ssh_session session)
    {
        int state, hlen;
        unsigned char *hash = NULL;
        char *hexa;
        char buf[10];
        state = ssh_is_server_known(session);
        hlen = ssh_get_pubkey_hash(session, &hash);
        if (hlen < 0)
            return -1;
        switch (state)
        {
        case SSH_SERVER_KNOWN_OK:
            printf("Server is known to be OK\n");
            break; /* ok */
        case SSH_SERVER_KNOWN_CHANGED:
            fprintf(stderr, "Host key for server changed: it is now:\n");
            ssh_print_hexa("Public key hash", hash, hlen);
            fprintf(stderr, "For security reasons, connection will be stopped\n");
            free(hash);
            return -1;
        case SSH_SERVER_FOUND_OTHER:
            fprintf(stderr, "The host key for this server was not found but an other"
                    "type of key exists.\n");
            fprintf(stderr, "An attacker might change the default server key to"
                    "confuse your client into thinking the key does not exist\n");
            free(hash);
            return -1;
        case SSH_SERVER_FILE_NOT_FOUND:
            fprintf(stderr, "Could not find known host file.\n");
            fprintf(stderr, "If you accept the host key here, the file will be"
                    "automatically created.\n");
            /* fallback to SSH_SERVER_NOT_KNOWN behavior */
        case SSH_SERVER_NOT_KNOWN:
            hexa = ssh_get_hexa(hash, hlen);
            fprintf(stderr,"The server is unknown. Do you trust the host key?\n");
            fprintf(stderr, "Public key hash: %s\n", hexa);
            free(hexa);
            if (fgets(buf, sizeof(buf), stdin) == NULL)
            {
                free(hash);
                return -1;
            }
            if (strncasecmp(buf, "yes", 3) != 0)
            {
                free(hash);
                return -1;
            }
            if (ssh_write_knownhost(session) < 0)
            {
                fprintf(stderr, "Error %s\n", strerror(errno));
                free(hash);
                return -1;
            }
            break;
        case SSH_SERVER_ERROR:
            fprintf(stderr, "Error %s", ssh_get_error(session));
            free(hash);
            return -1;
        }
        free(hash);
        return 0;
    }


} // namespace mpiclient


