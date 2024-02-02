use strict;
use warnings;
use File::Copy;
use File::stat;
use strict;
use DBI;
use POSIX qw(strftime);

my $relativeDir = "./";
my $cmd;
my $status;

print "Begin!!!\n";

print "-------------- 1.step: move_omdb_file --------------\n";
# 运行 move_omdb_file 需要当前程序工作目录存在一个文件: worldmanager3.nkvds
# 将原来按省分割的文件夹,变成按城市(省直管县/地级市/直辖市)分割的文件夹
# 接受2个参数
# 这行注释掉,单独执行
# $binary_path_folder/move_omdb_file /data/omdb-v1.5-23m3-hdhw /data/omdb-v1.5-23m3-hdhw-new 2>/dev/null
my $move_omdb_file_exe = "move_omdb_file.exe";
my $move_omdb_file_log = "move_omdb_file.log";
$cmd = join(' ', $move_omdb_file_exe, './omdb_file_directory', './omdb_file_directory_new', '1>', "$move_omdb_file_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$move_omdb_file_exe successfully\n";
}
else {
    print "$move_omdb_file_exe failed!\n";
}
print "\n\n";

print "-------------- 2.step: omdb_spatial_index --------------\n";
##构建网格空间索引
# $binary_path_folder/omdb_spatial_index --source-dir $1 --output-dir $2
my $omdb_spatial_index_exe = "omdb_spatial_index.exe";
my $omdb_spatial_index_log = "omdb_spatial_index.log";
$cmd = join(' ', $omdb_spatial_index_exe, '--sd', '1', '--source-dir', './omdb_file_directory_new', '--output-dir', './routing', '1>', "$omdb_spatial_index_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$omdb_spatial_index_exe successfully\n";
}
else {
    print "$omdb_spatial_index_exe failed\n";
}
print "\n\n";

print "-------------- 3.step: build_base_data --------------\n";
# 第一个参数为omdb_input_dir
my $build_base_data_exe = "build_base_data.exe";
my $build_base_data_log = "build_base_data.log";
$cmd = join(' ', $build_base_data_exe, './omdb_file_directory_new', '1>', "$build_base_data_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$build_base_data_exe successfully\n";
}
else {
    print "$build_base_data_exe failed\n"
}
print "\n\n";

print "-------------- 4.step: omdb render compiler and output rp file --------------\n";
my $omdb_exe = "omrc.exe";
my $omdb_log = "omrc.log";
$cmd = join(' ', $omdb_exe, 'omdb', '1>', "$omdb_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$build_base_data_exe successfully\n";
}
else {
    print "$build_base_data_exe failed\n"
}
print "\n\n";

print "-------------- 5.step: build_link_lane_data --------------\n";
# 第一个参数为omdb_input_dir
# 第二个参数为 rp 文件 所在目录
my $build_link_lane_exe = "build_link_lane_data.exe";
my $build_link_lane_log = "build_link_lane_data.log";
$cmd = join(' ', $build_link_lane_exe, './omdb_file_directory_new', "./routing", '1>', "$build_link_lane_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$build_link_lane_exe successfully\n";
}
else {
    print "$build_link_lane_exe failed\n";
}
print "\n\n";

print "-------------- 6.step: merge_nkvd_table  --------------\n";
# 第一个参数为omdb_input_dir
# 第二个参数为 hlq.db 所在目录
my $merge_nkvd_table_exe = "merge_nkvd_table.exe";
my $merge_nkvd_table_log = "merge_nkvd_table.log";
$cmd = join(' ', $merge_nkvd_table_exe, './omdb_file_directory_new', "./routing", '1>', "$merge_nkvd_table_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$merge_nkvd_table_exe successfully\n";
}
else {
    print "$merge_nkvd_table_exe failed\n";
}
print "\n\n";

print "-------------- 7.step: merge_nkvd_to_sqlite --------------\n";
# Step 5
# 第一个参数为omdb_input_dir
my $merge_nkvd_to_sqlite_exe = "merge_nkvd_to_sqlite.exe";
my $merge_nkvd_to_sqlite_log = "merge_nkvd_to_sqlite.log";
$cmd = join(' ', $merge_nkvd_to_sqlite_exe, './omdb_file_directory_new', '1>', "$merge_nkvd_to_sqlite_log", '2>&1');
$status = system($cmd);
print "\n";
if ($status == 0) {
    print "$merge_nkvd_to_sqlite_exe successfully\n";
}
else {
    print "$merge_nkvd_to_sqlite_exe failed\n";
}
print "\n\n";

print "-------------- 8.step: db_file_merge --------------\n";
system('del /q ./china_refmap.db'); #windows
copy("./output-d/china_refmap.db", "./china_refmap.db") or die "Copy failed: $!";
my $db_file = './china_refmap.db';
my $db1_file = './china-route.db';
my $dbh = DBI->connect("dbi:SQLite:dbname=$db_file", "", "") or die "failed to connect database: $DBI::errstr";
$dbh->do("ATTACH DATABASE '$db1_file' AS 'db1'") or die "failed to attach database: " . $dbh->errstr;
my $table_name = 'Routing';
my $sth = $dbh->prepare("SELECT name FROM sqlite_master WHERE type='table' AND name=?");
$sth->execute($table_name) or die "failed to query: " . $sth->errstr;
my $result = $sth->fetchrow_array();
if (defined $result) {
    print "table $table_name exists in database\n";
} else {
    print "table $table_name not exists in database\n";
    my $create_table_sql = "CREATE TABLE IF NOT EXISTS $table_name (id INTEGER PRIMARY KEY NOT NULL, data BLOB )";
    $dbh->do($create_table_sql) or die "failed to create table: " . $dbh->errstr;
    my $insert_sql = "INSERT INTO $table_name SELECT * FROM db1.T";
    $dbh->do($insert_sql) or die "failed to insert data: " . $dbh->errstr;
}

my $tableMeta = '__Routing_meta__';
my $originMeta = "__Had_meta__";
my $current_time = time();
my ($sec, $min, $hour, $mday, $mon, $year) = localtime($current_time);
$mon += 1;
$year += 1900;
my $formatted_date = strftime("%Y%m%d%H%M", $sec, $min, $hour, $mday, $mon-1, $year-1900);
$sth = $dbh->prepare("SELECT COUNT(*) FROM $originMeta WHERE key = ?");
$sth->execute("ncVersion");
my ($count) = $sth->fetchrow_array();
if ($count == 0) {
    print "ncVersion not exists\n";
    my $sqlString = "INSERT INTO $originMeta (key, value) VALUES ('ncVersion', $formatted_date)";
    $dbh->do($sqlString) or die "failed to create table: " . $dbh->errstr;
} else {
    print "ncVersion exists\n";
}

$sth = $dbh->prepare("SELECT name FROM sqlite_master WHERE type='table' AND name=?");
$sth->execute($tableMeta) or die "failed to query: " . $sth->errstr;
my $resultMeta = $sth->fetchrow_array();
if (defined $resultMeta) {
    print "table $tableMeta exists in database\n";
} else {
    print "table $tableMeta not exists in database\n";
    my $create_table_sql = "CREATE TABLE $tableMeta AS SELECT * FROM $originMeta";
    $dbh->do($create_table_sql) or die "failed to create table: " . $dbh->errstr;
}

$sth->finish();
$dbh->disconnect();
print "db merge successfully\n";
print "\n\n";

print "Finished!!!\n";
system('pause');

