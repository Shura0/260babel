#!/usr/bin/perl -w
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
# Author:       Alexander Zaitsev <shura0@yandex.ru>
# Some pieces are written by Niccolo Rigacci <niccolo@rigacci.org>
#
# Version:      0.3.3   2014-10-05
#
# Control program for GPS logger Holux GPSport260

use Date::Format;
use Date::Parse;
use Device::SerialPort;
use File::Basename;
use Getopt::Std;
use utf8;
use strict;

my $NAME = basename($0);
use vars qw($opt_b $opt_B $opt_d $opt_f $opt_h $opt_l $opt_m $opt_p $opt_r $opt_s $opt_u $opt_v);


# For tracks list
my $LIST_NAME_OFFSET=4;
my $LIST_TIME_OFFSET=16;
my $LIST_LENGTH_OFFSET=24;
my $LIST_MEM_START_OFFSET=28;
my $LIST_MEM_LENGTH_OFFSET=32;
my $LIST_NAME_LENGTH=11;

#For track file
my $TRACK_TIME_OFFSET=0;
my $TRACK_LAT_OFFSET=4;
my $TRACK_LON_OFFSET=8;
my $TRACK_HEIGHT_OFFSET=12;
my $TRACK_SPEED_OFFSET=14;
my $TRACK_FLAGS_OFFSET=17;
my $TRACK_HEARTRATE_OFFSET=18;
my $TRACK_ALTIMETER_OFFSET=20;
my $TRACK_HEADING_OFFSET=22;
my $TRACK_DISTANCE_OFFSET=24;


#Upload track
my $PLACEMARK_DELIMETER=0x7c910060;
my $PLACEMARK_EMPTY=0x05ba1760;
my $MAX_PLACEMARK_NAME=14;
my $MAX_TRACK_NAME=18;
my $MAX_SEGMENT_SIZE=512;

my $GPX_EOL="\n";

my $ret;
my $bytes_to_read;

my $TIMEOUT = 5;
# Timeout waiting for char "$" which is MTK packet start (sec).
my $TIMEOUT_PKT_PREAMBLE = 20;
# Timeout for activity on device port (msec).
my $TIMEOUT_IDLE_PORT = 5000;

#key for checksum
my $KEY=pack "H*",'00000000B71DC1046E3B8209D926430DDC7604136B6BC517B24D861A0550471EB8ED08260FF0C922D6D68A2F61CB4B2B649B0C35D386CD310AA08E3CBDBD4F3870DB114CC7C6D0481EE09345A9FD5241ACAD155F1BB0D45BC2969756758B5652C836196A7F2BD86EA60D9B6311105A6714401D79A35DDC7D7A7B9F70CD665E74E0B6239857ABE29C8E8DA191399060953CC0278B8BDDE68F52FBA582E5E66486585B2BBEEF46EABA3660A9B7817D68B3842D2FAD3330EEA9EA16ADA45D0B6CA0906D32D42770F3D0FE56B0DD494B71D94C1B36C7FB06F7C32220B4CE953D75CA28803AF29F9DFBF646BBB8FBF1A679FFF4F63EE143EBFFE59ACDBCE82DD07DEC77708634C06D4730194B043DAE56C539AB0682271C1B4323C53D002E7220C12ACF9D8E1278804F16A1A60C1B16BBCD1F13EB8A01A4F64B057DD00808CACDC90C07AB9778B0B6567C69901571DE8DD475DBDD936B6CC0526FB5E6116202FBD066BF469F5E085B5E5AD17D1D576660DC5363309B4DD42D5A490D0B1944BA16D84097C6A5AC20DB64A8F9FD27A54EE0E6A14BB0A1BFFCAD60BB258B23B69296E2B22F2BAD8A98366C8E41102F83F60DEE87F35DA9994440689D9D662B902A7BEA94E71DB4E0500075E4892636E93E3BF7ED3B6BB0F38C7671F7555032FAE24DF3FE5FF0BCC6E8ED7DC231CB3ECF86D6FFCB8386B8D5349B79D1EDBD3ADC5AA0FBD8EEE00C6959FDCD6D80DB8E6037C64F643296087A858BC97E5CAD8A73EBB04B77560D044FE110C54B383686468F2B47428A7B005C3D66C158E4408255535D43519E3B1D252926DC21F0009F2C471D5E28424D1936F550D8322C769B3F9B6B5A3B26D6150391CBD40748ED970AFFF0560EFAA011104DBDD014949B93192386521D0E562FF1B94BEEF5606DADF8D7706CFCD2202BE2653DEAE6BC1BA9EB0B0668EFB6BB27D701A6E6D3D880A5DE6F9D64DA6ACD23C4DDD0E2C004F6A1CDB3EB60C97E8D3EBDC990FFB910B6BCB4A7AB7DB0A2FB3AAE15E6FBAACCC0B8A77BDD79A3C660369B717DF79FA85BB4921F4675961A163288AD0BF38C742DB081C330718599908A5D2E8D4B59F7AB085440B6C95045E68E4EF2FB4F4A2BDD0C479CC0CD43217D827B9660437F4F460072F85BC176FD0B86684A16476C93300461242DC565E94B9B115E565A1587701918306DD81C353D9F0282205E065B061D0BEC1BDC0F51A69337E6BB52333F9D113E8880D03A8DD097243ACD5620E3EB152D54F6D4297926A9C5CE3B68C1171D2BCCA000EAC8A550ADD6124D6CD2CB6B2FDF7C76EEDBC1CBA1E376D660E7AFF023EA18EDE2EE1DBDA5F0AAA064F4738627F9C49BE6FD09FDB889BEE0798D67C63A80D0DBFB84D58BBC9A62967D9EBBB03E930CADFF97B110B0AF060D71ABDF2B32A66836F3A26D66B4BCDA7B75B8035D36B5B440F7B1';


my $port     = '/dev/ttyUSB0';   # Default communication port.
my $baudrate = 38400;           # Default port speed.
my $device;

my $PI=3.1415926535;

my $TRACKS_TO_GET;
my @TRACKLIST;
my @POI;
my $BBOX={};
my @WPT;
my @TRACK;


if (! getopts('bB:d:f:hlmp:rs:u:v') or $opt_h or !($opt_u or $opt_l or $opt_d or $opt_r or $opt_B)) {
	print <<HELP;
Usage: $NAME [options]
Options:
	-b                       Also save raw data in .bin file
	-B <filename>            Read binary file instead of device
	-d <tracklist>           List tracks to get, comma separated.
                             Special values: a - all tracks
                                             l - last track
                                             Example: -d 1,2,3,5,6 or -d l
	-f <filename>            Base name for saved files (.bin and .gpx)
	-h                       Print this message and exit
	-l                       Print track list
	-m                       Save tracks in one file
	-p <port>                Communication port, default: $port
	-r                       Remove all data
	-s <speed>               Serial port speed, default $baudrate baud
	-u <filename>            Upload GPX file to device.

Example:
	Download last track and waypoints from GPS device

	260babel -f gpsdata -d l

HELP
	exit(1)
}

if ($opt_v) { print "\n260Babel Version 0.3.3\n\n"; exit }
if ($opt_m and $opt_b) { print "\nImpossible to use -m and -b in the same time\n\n";exit }
	

if($opt_B) {
	open my $f,"<$opt_B" or die $!;
	binmode $f;
	my @d=<$f>;
	my $d=join '',@d;
	close $f;
	my @points=parse_track($d);
	my $name=$opt_f||$opt_B;
	my $bbox=get_bounds({data=>\@points});
	my @poi=get_poi(\@points);
	my $gpx=generate_gpx_header($bbox);
	$gpx.=generate_gpx_poi(@poi);
	$gpx.=generate_gpx_track({name=>$name,data=>\@points});
	$gpx.=generate_gpx_footer();
	open($f, ">$name.gpx") or die $!;
	print $f $gpx;
	close $f;
	exit 0;
}

$TRACKS_TO_GET=$opt_d if($opt_d);
$baudrate = int($opt_s) if (defined($opt_s));
serial_port_open($port, $baudrate);

# Init
packet_send('PHLX810');
packet_wait('PHLX852,GR260');
packet_send('PHLX832');
packet_wait('PHLX865,');
packet_send('PHLX829');
$ret=packet_wait('PHLX861,');
print "Firmware version: $1.$2\n" if($ret=~m/PHLX861,(\d)(\d+)/);
packet_send('PHLX826'); #turn on USB icon.
$ret=packet_wait('PHLX859'); #usb icon is displayed
if($ret=~m/PHLX859/)
{
	serial_port_speed('921600');
	sleep 1;
}

if($opt_u) {
	open_gpx_file($opt_u);
	my $placemarks=generate_placemarks_bin();
	my $trackdata=generate_track_bin();
	my $placemarks_crc=calc_crc($placemarks);
	my $trackdata_crc=calc_crc($trackdata);
	
	packet_send('PHLX704,1,2');
	packet_wait('PHLX604,');
	
	my $pl_length=length $placemarks;
	my $to_send=sprintf("PHLX901,%u,%X",$pl_length,$placemarks_crc);
	packet_send($to_send);# Full size of data
	packet_wait('PHLX900,901,3');#ok
	
	for(my $i=0;$i<$pl_length;$i+=$MAX_SEGMENT_SIZE)
	{
		my $ss=0;
		if (($pl_length-$i)<$MAX_SEGMENT_SIZE) {
			$ss=$pl_length-$i;
		}
		else
		{
			$ss=$MAX_SEGMENT_SIZE;
		}
		my $d=substr($placemarks,$i,$ss);
		my $crc=calc_crc($d);
		$to_send=sprintf("PHLX902,%u,%d,%X",$i,$ss,$crc);
		packet_send($to_send);
		packet_wait('PHLX900,902,3'); #ready to receive
		raw_packet_send($d);
	}
	packet_wait('PHLX900,902,3'); #confirm
	sleep 1;
	packet_send('PHLX708');
	packet_send('PHLX708');
	packet_wait('PHLX900,708,3');
	
	my $tr_length=length $trackdata;
	
	$to_send=sprintf("PHLX901,%u,%X",$tr_length,$trackdata_crc);
	
	packet_send($to_send);# Full size of data
	packet_wait('PHLX900,901,3');#ok
	for(my $i=0;$i<$tr_length;$i+=$MAX_SEGMENT_SIZE)
	{
		my $ss=0;
		if (($tr_length-$i)<$MAX_SEGMENT_SIZE) {
			$ss=$tr_length-$i;
		}
		else
		{
			$ss=$MAX_SEGMENT_SIZE;
		}
		my $d=substr($trackdata,$i,$ss);
		my $crc=calc_crc($d);
		$to_send=sprintf("PHLX902,%u,%u,%X",$i,$ss,$crc);
		packet_send($to_send);
		packet_wait('PHLX900,902,3'); #ready to receive
		raw_packet_send($d);
		packet_wait('PHLX900,902,3'); #ok
	}
	sleep 1;
	
	print "Sucessfully uploaded\n";
	
	#packet_wait('PHLX900,902,3'); #ready to receive
	#packet_send('PHLX902,0,256,B699765A');#Fist segment of data
	#packet_wait('PHLX900,902,3'); #ready to receive
	#send binary data; (global point)
	#packet_wait('PHLX900,902,3'); #confirm
	#packet_send('PHLX708');
	#packet_send('PHLX708');
	#packet_wait('PHLX900,708,3');
	#packet_send('PHLX901,296,8A2BDDBB');# Full size of data
	#packet_wait('PHLX900,901,3');#ok
	#packet_send('PHLX902,0,296,8A2BDDBB');#Fist segment of data
	#packet_wait('PHLX900,902,3'); #ready to receive
	#send binary data; (track data)
	#packet_wait('PHLX900,902,3'); #confirm
	#return
}

if($opt_d or $opt_l)
{
	packet_send('PHLX701');
	$ret=packet_wait('PHLX601,');
	if($ret=~m/PHLX601,(\d+)/) # saved tracks PHLX601,
	{
		# Read tracklist
		if($1==0)
		{
			print "No tracks!\n";
			sleep 1;
			packet_send('PHLX827'); #Remove USB icon
			packet_wait('PHLX860'); #icon removed
			exit 0;
		}
		packet_send("PHLX702,0,$1");
		packet_wait('PHLX900,702,');
		$ret=packet_wait('PHLX901,');
		if($ret=~m/PHLX901,(\d+)/)
		{
			$bytes_to_read=$1;
			my $d=read_binary_data($bytes_to_read);
			parse_track_list($d) if($d);
			sleep 2;
			packet_send('PHLX831');
			packet_wait('PHLX863,');
			sleep 1;
			if($opt_l)
			{
				packet_send('PHLX827'); #Remove USB icon
				packet_wait('PHLX860'); #icon removed
				my $i=1;
				foreach my $a (@TRACKLIST)
				{
					my $date=localtime($a->{info}->{time});
					printf("%d - %s\t Len: %i m \tName: %s\n", $i++,$date,$a->{info}->{lenght},$a->{info}->{name});
				}
				exit 0;
			}

			#get tracks
			my @ttg;
			if ($TRACKS_TO_GET=~/a/) {
				@ttg=('a');
			}
			else
			{
				@ttg=split ',',$TRACKS_TO_GET;
			}
			my $data;
			foreach my $t (@ttg)
			{
				if($t=~/\d+/ and $t>0)
				{
					if($t>0)
					{
						get_track($t-1);#because track indexes start with 0.
					}
				}
				elsif($t eq 'l')
				{
					$t=scalar (@TRACKLIST)-1;
					get_track($t);
				}
				elsif($t eq 'a')
				{
					for(my $i=0;$i<scalar @TRACKLIST;$i++)
					{
					   get_track($i);
					}
				}
				else
				{
					die "Urecognized -d option argument";
				}
			}
			save_tracks();
		}
	}
}

if($opt_r)
{
	print "Removing tracks...";
	sleep 1;
	packet_send('PHLX839');
	packet_wait('PHLX870,', 60);
	print "ok\n";
}
sleep 3;
packet_send('PHLX827'); #Remove USB icon
packet_wait('PHLX860'); #icon removed


sub serial_port_open {
	my $port = shift;
	my $baudrate = shift;
	die("Cannot open $port. Did you switch ON the GPS device?\n") if (! -c $port);
	$device = Device::SerialPort->new($port)
		|| die "ERROR: Opening serial device $port: $!";
	$device->baudrate($baudrate) || die "fail setting baud rate";
	$device->parity('none')      || die "fail setting parity";
	$device->databits(8)         || die "fail setting databits";
	$device->stopbits(1)         || die "fail setting stopbits";
	$device->handshake('none')   || die "fail setting handshake";
	$device->write_settings      || die "no settings";
}

# Close the port.
sub serial_port_close {
	$device->close || warn "close failed";
}

sub serial_port_write {
	return($device->write(shift));
}

# Set read timeout (in milliseconds) on the port.
sub serial_port_set_read_timeout {
	$device->read_const_time(shift);
}

sub serial_port_getch {
	return($device->read(1));
}

sub serial_port_speed {
	$device->baudrate(shift) || die "fail setting baud rate";
}

sub packet_send {

	my $pkt = shift;
	my $n;

	# Add the checksum to the packet.
	$pkt = $pkt . '*' . sprintf('%02X', packet_checksum($pkt));
	#printf("%s TX packet => %s\n", log_time(), $pkt) if ($debug >= $LOG_NOTICE);
	# Add the preamble and <CR><LF>.
	$pkt = '$' . $pkt . "\r\n";

	$n = serial_port_write($pkt);
	#printf("Writing %u bytes to device; actually written %u bytes\n", length($pkt), $n) if ($debug >= $LOG_DEBUG);
	die("ERROR: Writing to device: $!") if ($n != length($pkt));
}

sub raw_packet_send {

	my $pkt = shift;
	my $n;
	$n = serial_port_write($pkt);
	#printf("Writing %u bytes to device; actually written %u bytes\n", length($pkt), $n) if ($debug >= $LOG_DEBUG);
	die("ERROR: Writing to device: $!") if ($n != length($pkt));
}

#-------------------------------------------------------------------------
# Calculate the packet checksum: bitwise XOR of string's bytes.
#-------------------------------------------------------------------------
sub packet_checksum {

	my $pkt   = shift;
	my $len   = length($pkt);
	my $check = 0;
	my $i;

	for ($i = 0; $i < $len; $i++) { $check ^= ord(substr($pkt, $i, 1)); }
	#printf("0x%02X\n", $check);
	return($check);
}


#-------------------------------------------------------------------------
# Read a packet from the device.
# Return the packet with PktType, DataField, "*" and Checksum.
#
#   Example: PMTK182,3,8,0004E69C*13
#
# The packet received has a leading Preample and a trailing <CR><LF>,
# example: $PMTK182,3,8,0004E69C*13<CR><LF>
#-------------------------------------------------------------------------
sub packet_read {

	my $timeout = shift;
	my $c;
	my $n;
	my $t;
	my $pkt;
	my $previous_c;
	my $payload;
	my $checksum;

	# Timeout (in milliseconds) for activity on the port.
	$timeout = $TIMEOUT_IDLE_PORT if (!defined($timeout));
	serial_port_set_read_timeout($timeout);

	# Wait packet preamble.
	$c = '';
	$t = time();
	while ($c ne '$' and (time() - $t) < $TIMEOUT_PKT_PREAMBLE) {
		($n, $c) = serial_port_getch();
		die("ERROR: Reading from device (may be switched OFF)")if ($n != 1);
	}
	die("ERROR: Packet preamble not found (wrong serial speed or output from device not flushed)\n") if ($c ne '$');

	# Read until End Of Packet.
	$pkt = '';
	$previous_c = '';
	while (1) {
		($n, $c) = serial_port_getch();
		die("ERROR: Reading from device (may be switched OFF): $!") if ($n != 1);
		if ($c eq '$') {
			$pkt = '';
		} else {
			$pkt .= $c;
		}
		if (($c eq "\n") and ($previous_c eq "\r")) {
			last;
		}
		$previous_c = $c;
	}

	# Remove trailing <CR><LF>.
	$pkt = substr($pkt, 0, -2);
	#printf("%s RX packet <= %s\n", log_time(), $pkt) if ($debug >= $LOG_NOTICE);

	# Extract packet payload and checksum.
	$payload  = substr($pkt,  0, -3);
	$checksum = hex(substr($pkt, -2,  2));

	# Verify packet checksum.
	if ($checksum ne packet_checksum($payload)) {
		#printf("Packet checksum error: expected 0x%02X, computed 0x%02X\n", $checksum, packet_checksum($payload)) if ($debug >= $LOG_ERR);
		return('');
	} else {
		return($pkt);
	}
}


sub raw_packet_read {

	my $len=shift;
	my $timeout = shift;
	my $count=0;
	my $c;
	my $n;
	my $t;
	my $pkt;

	# Timeout (in milliseconds) for activity on the port.
	$timeout = $TIMEOUT_IDLE_PORT if (!defined($timeout));
	serial_port_set_read_timeout($timeout);
	$c = '';
	$t = time();
	while ($count<$len and (time() - $t) < $TIMEOUT_PKT_PREAMBLE) {
		($n, $c) = serial_port_getch();
		$count+=$n;
		die("ERROR: Reading from device (may be switched OFF)")if (($n != 1) and ( $count < $len ) );
		$pkt.=$c;
	}
	return($pkt);
}


#-------------------------------------------------------------------------
# Read packets from the device, untill we get the one we want.
#-------------------------------------------------------------------------
sub packet_wait {

	my $pkt_type = shift;
	my $timeout  = shift;
	my $max_time;
	my $pkt;
	my $len;
	my $i;

	$len = length($pkt_type);

	# Timeout (in seconds) for packet wait.
	$timeout = $TIMEOUT if (!defined($timeout));
	$max_time = time() + $timeout;

	while(1) {
		$pkt = packet_read($timeout * 1000);
		return($pkt) if (substr($pkt, 0, $len) eq $pkt_type);
		#write_log_packet($pkt) if (defined($fp_log));
		last if (time() > $max_time);
	}
	#printf("%s ERROR: packet_wait() failed for packet %s\n", log_time(), $pkt_type) if ($debug >= $LOG_ERR);
	return(undef);
}



sub parse_track_list
{
	my $d=shift;
	die "File length is wrong!\n" if(length($d)%64);
	my $tracks=length($d)/64;
	print "Found $tracks track(s)\n";
	my $c;
	for(my $i=0;$i<$tracks;$i++)
	{
		my $track={
				   info=>{}
				   };
		my $s=$i*64;
		$c=substr($d,$s+3,1);
		if(ord($c)==0xff)
		{
			if(ord(substr($d,$s+$LIST_NAME_OFFSET,1))==0xff)
			{
				$track->{info}->{'name'}="Noname";
			}
			else
			{
				$track->{info}->{'name'}=substr($d,$s+$LIST_NAME_OFFSET,$LIST_NAME_LENGTH);
				$track->{info}->{'name'}=~ s/[^[:print:]]+//g;
			}
			# MTK time starts in Y2000, Unix in 1970. It's 30 years shift.
			$track->{info}->{'time'}=unpack('V',substr($d,$s+$LIST_TIME_OFFSET,4))+946684800;
			$track->{info}->{'lenght'}=unpack('V',substr($d,$s+$LIST_LENGTH_OFFSET,4));
			$track->{info}->{'mem_start'}=unpack('V',substr($d,$s+$LIST_MEM_START_OFFSET,4));
			$track->{info}->{'mem_length'}=unpack('V',substr($d,$s+$LIST_MEM_LENGTH_OFFSET,4));
			push @TRACKLIST,$track;
		}
	}
}


sub read_binary_data
{
	
	my $d;
	my $data;
	my $bytes_to_read=shift;
	my $bytes_left=$bytes_to_read;
	#print "Bytes to read: $bytes_to_read\n";
	packet_send('PHLX900,901,3');
	
	my $fails=0;
	while (1){
		$ret=packet_wait('PHLX902,');
		if($ret=~m/PHLX902,\d+,(\d+),([^\*]+)/)
		{
			packet_send('PHLX900,902,3');
			my $segment_size=$1;
			my $crc=$2;
			$d=raw_packet_read($segment_size);
			if (check_crc($d,$crc)) {
				$bytes_left-=$segment_size;
				packet_send('PHLX900,902,3');
				$data.=$d;
				$fails=0;
				last if ($bytes_left<1);
			}
			else #request to retransmit last data block
			{
				packet_send('PHLX900,902,2');
				return undef if ($fails++>1);
			}
		}
	}
	sleep 2;
	return $data;
}

sub get_track {
	my $t=shift; #track number in list
	return undef if(!($t=~/\d+/ and $t>-1));

	# Get Tracks
	#                    shift 28 in track list
	#                     |   shift 32 in track list
	#                     |    |
	#packet_send('PHLX703,223,63');
	die "No such track" if (!$TRACKLIST[$t]);
	packet_send('PHLX703,'.$TRACKLIST[$t]->{info}->{mem_start}.','.$TRACKLIST[$t]->{info}->{mem_length});
	packet_wait('PHLX900,703,');
	$ret=packet_wait('PHLX901,');
	if($ret=~m/PHLX901,(\d+)/)
	{
		$bytes_to_read=$1;
		my $d=read_binary_data($bytes_to_read);
		if ($d && length($d)==$bytes_to_read)
		{
			if ($opt_b) {
				$TRACKLIST[$t]->{rawdata}=$d;
			}
			my @points=parse_track($d);
			$TRACKLIST[$t]->{data}=\@points;
			if($opt_b)
			{
				my $filename;
				$filename=$opt_f."_" if($opt_f);
				my $a=$TRACKLIST[$t];
				if($a->{info}->{name} ne 'Noname')
				{
					$filename.=$a->{info}->{name};
				}
				else
				{
					$filename.=filename_time($a->{info}->{time});
				}
				open my $ff, ">$filename.bin" or die $!;
				binmode $ff;
				print $ff $d;
				close $ff;
			}
		}
		else
		{
			print "Failed to read track. CRC error\n";
		}
	}
}


sub parse_track
{
	# point is 32 bytes length. I don't know what means some fields.
	#
	#          time         LAT          LON        height  speed*36
	#          _|_________ __|________  __|________ __|__ __|__
	#00000000  c2 0d b0 1b 72 0a 60 42  bb 49 17 42 b3 00 12 00
	#00000010  ea 05 00 00 a9 00 01 00  00 00 00 00 00 00 00 00
	#             ^^ ^^^^^ ^^^^^ ^^^^^  ^^^^^^^^^^^
	#             |   |      |    |       |
	#             | heartrate|   heading  distance
	#             flags      altimeter
	my $d=shift;
	my @track;
	die "File has wrong size" if(length($d)%32);
	for(my $i=0;$i<length($d);$i+=32)
	{
		my $point={};
		#MTK time starts in Y2000, Unix in 1970. It's 30 years shift.
		$point->{time}=unpack('V',substr($d,$i+$TRACK_TIME_OFFSET,4))+946684800;
		$point->{lat}=unpack('f', substr($d,$i+$TRACK_LAT_OFFSET,4));
		$point->{lon}=unpack('f',substr($d,$i+$TRACK_LON_OFFSET,4));
		$point->{height}=unpack('v', substr($d,$i+$TRACK_HEIGHT_OFFSET,2));
		$point->{speed}=unpack('v',substr($d,$i+$TRACK_SPEED_OFFSET,2));
		$point->{flags}=unpack('C',substr($d,$i+$TRACK_FLAGS_OFFSET,1));
		$point->{heart}=unpack('v',substr($d,$i+$TRACK_HEARTRATE_OFFSET,2));
		$point->{alt}=unpack('v',substr($d,$i+$TRACK_ALTIMETER_OFFSET,2));
		$point->{heading}=unpack('v',substr($d,$i+$TRACK_HEADING_OFFSET,2));
		$point->{distance}=unpack('V',substr($d,$i+$TRACK_DISTANCE_OFFSET,4));
		push @track, $point;
	}
	return @track;
}


sub save_tracks
{
	my $GPX_TEXT;
	if ($opt_m)
	{
		my $index=0;
		my $bbox=undef;
		my @poi;
		my $text;
		my $filename=$opt_f || "output";
		print "Saving all the tracks to ".$filename.".gpx\n";
		foreach my $a (@TRACKLIST)
		{
			next if !defined $a->{data};
			$bbox=get_bounds({data=>$a->{data},
							  bbox=>$bbox});
			push @poi,get_poi($a->{data});
		}
		$text=generate_gpx_header($bbox);
		$text.=generate_gpx_poi(@poi);	
		foreach my $a (@TRACKLIST)
		{
			next if ! defined $a->{data};
			$index++;
			my $name=$a->{info}->{name};
			$name=utc_time($a->{info}->{time}) if ($name eq "Noname");
			
			$text.=generate_gpx_track({number=>$index,
									   name=>$name,
									   data=>$a->{data}});
		}
		$text.=generate_gpx_footer();
		open my $fh, ">$filename.gpx" or die $!;
		printf $fh $text;
		close $fh;
		print "ok\n\n";
		
	}
	else
	{
		foreach my $a (@TRACKLIST)
		{
			next if !defined $a->{data};
			my $filename;
			$filename=$opt_f."_" if($opt_f);
			if($a->{info}->{name} ne 'Noname')
			{
				$filename.=$a->{info}->{name};
			}
			else
			{
				$filename.=filename_time($a->{info}->{time});
			}
			$a->{info}->{filename}=$filename;
			print "Saving track to ".$filename.".gpx\n";
			
			my $bbox=get_bounds({data=>$a->{data}});
			my @poi=get_poi($a->{data});
			my $text=generate_gpx_header($bbox);
			$text.=generate_gpx_poi(@poi);
			my $name=$a->{info}->{name};
			$name=utc_time($a->{info}->{time}) if ($name eq "Noname");
			$text.=generate_gpx_track({number=>1,
									   name=>$name,
									   data=>$a->{data}});
			$text.=generate_gpx_footer();
			open my $fh, ">$filename.gpx" or die $!;
			printf $fh $text;
			close $fh;
			print "ok\n\n";
		}
	}
}

sub generate_gpx_header
{
	my $bbox=shift;
	my $gpx=sprintf('<?xml version="1.0" encoding="UTF-8"?>%s', $GPX_EOL);
	$gpx.='<gpx' . $GPX_EOL;
	$gpx.='  version="1.1"' . $GPX_EOL;
	$gpx.='  creator="MTKBabel - http://www.rigacci.org/"' . $GPX_EOL;
	$gpx.='  xmlns="http://www.topografix.com/GPX/1/1"' . $GPX_EOL;
	$gpx.='  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"' . $GPX_EOL;
	$gpx.='  xmlns:mtk="http://www.rigacci.org/gpx/MtkExtensions/v1"' . $GPX_EOL;
	$gpx.='  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd' . $GPX_EOL;
	$gpx.='                      http://www.rigacci.org/gpx/MtkExtensions/v1 http://www.rigacci.org/gpx/MtkExtensions/v1/MtkExtensionsv1.xsd">' . $GPX_EOL;
	$gpx.=sprintf('<metadata>%s', $GPX_EOL);
	$gpx.=sprintf('  <time>%s</time>%s', utc_time(time()), $GPX_EOL);
	$gpx.=sprintf('  <bounds minlat="%.9f" minlon="%.9f" maxlat="%.9f" maxlon="%.9f"/>%s',
				  $bbox->{minlat}, $bbox->{minlon}, $bbox->{maxlat}, $bbox->{maxlon}, $GPX_EOL) if(defined($bbox));
	$gpx.=sprintf('</metadata>%s', $GPX_EOL);
	return $gpx;
}

sub generate_gpx_poi
{
	my @poi=@_;
	my $poi_index=0;
	my $gpx='';
	foreach my $a (@poi)
	{
		next if ! defined $a->{lon} || ! defined $a->{lat};
		$poi_index++;
		my $record_utc=utc_time($a->{time});
		$gpx.=sprintf('<wpt lat="%.9f" lon="%.9f">%s', $a->{lat}, $a->{lon}, $GPX_EOL);
		$gpx.=sprintf('  <ele>%.6f</ele>%s',   $a->{height}, $GPX_EOL) if (defined($a->{height}));
		$gpx.=sprintf('  <time>%s</time>%s',   $record_utc,    $GPX_EOL) if (defined($record_utc));
		$gpx.=sprintf('  <name>%03d</name>%s', $poi_index, $GPX_EOL);
		$gpx.=sprintf('  <cmt>%03d</cmt>%s',   $poi_index, $GPX_EOL);
		$gpx.=sprintf('  <desc>%s</desc>%s',   $record_utc,     $GPX_EOL) if (defined($record_utc));
		$gpx.=sprintf('  <sym>Flag</sym>%s',                    $GPX_EOL);
		$gpx.=sprintf('</wpt>%s', $GPX_EOL);
	}
	return $gpx;
}

sub generate_gpx_track
{
	my ($args)=@_;
	my $gpx_trk_number=$args->{number}||1;
	my $name=$args->{name};
	my @data=@{$args->{data}};
	my $gpx;
	$gpx.=sprintf('<trk>%s', $GPX_EOL);
	$gpx.=sprintf('  <name>%s</name>%s', $name, $GPX_EOL);
	$gpx.=sprintf('  <number>%u</number>%s', $gpx_trk_number, $GPX_EOL) if ($gpx_trk_number > 0);
	$gpx.=sprintf('<trkseg>%s', $GPX_EOL);
	foreach my $a (@data)
	{
		my $record_latitude=$a->{lat};
		my $record_longitude=$a->{lon};
		my $record_height=$a->{alt}; #height from Altimeter data
		my $record_utc=utc_time($a->{time});
		my $record_speed=$a->{speed}/36;
		my $record_heading=$a->{heading};
		$gpx.=sprintf('<trkpt lat="%.9f" lon="%.9f">%s', $record_latitude, $record_longitude, $GPX_EOL);
		$gpx.=sprintf('  <ele>%.6f</ele>%s', $record_height, $GPX_EOL) if (defined($record_height));
		$gpx.=sprintf('  <time>%s</time>%s', $record_utc,    $GPX_EOL) if (defined($record_utc));
		$gpx.=sprintf('  <course>%03d</course>%s', $record_heading,    $GPX_EOL) if (defined($record_heading));
		$gpx.=sprintf('  <speed>%.2f</speed>%s', $record_speed,    $GPX_EOL) if (defined($record_speed));
		if ($a->{heart}) {
			$gpx.=sprintf('  <extensions>%s',$GPX_EOL);
			$gpx.=sprintf('   <gpxtpx:TrackPointExtension>%s',$GPX_EOL);
			$gpx.=sprintf('    <gpxtpx:hr>%d</gpxtpx:hr>%s',$a->{heart}, $GPX_EOL);
			$gpx.=sprintf('   </gpxtpx:TrackPointExtension>%s',$GPX_EOL);
			$gpx.=sprintf('  </extensions>%s',$GPX_EOL);
		}
		$gpx.=sprintf('</trkpt>%s', $GPX_EOL);
	}
	$gpx.=sprintf('</trkseg>%s', $GPX_EOL);
	$gpx.=sprintf('</trk>%s', $GPX_EOL);
	return $gpx;
}

sub generate_gpx_footer
{
	my $gpx;
	$gpx.=sprintf('</gpx>%s', $GPX_EOL);
	return $gpx;
}

sub get_poi
{
	my $data=shift;
	my @poi;
	foreach my $a (@{$data})
	{
		if($a->{flags}>=0x10) #POI
		{
			my $poi={};
			$poi->{lon}=$a->{lon};
			$poi->{lat}=$a->{lat};
			$poi->{height}=$a->{alt};
			$poi->{time}=$a->{time};
			$poi->{name}=localtime($a->{time});
			$poi->{description}=$poi->{name};
			push @poi, $poi;
		}
	}
	return @poi;	
}

sub get_bounds
{
	my ($args)=@_;
	my $bbox=$args->{bbox}||{};
	my @data=@{$args->{data}};
	$bbox->{minlat}=$bbox->{maxlat}=$data[0]->{lat} if(!defined($bbox->{maxlat}));
	$bbox->{minlon}=$bbox->{maxlon}=$data[0]->{lon} if(!defined($bbox->{maxlon}));
	foreach my $a (@data)
	{
		my $poi={};
		$bbox->{minlon}=$a->{lon} if($a->{lon}<$bbox->{minlon});
		$bbox->{maxlon}=$a->{lon} if($a->{lon}>$bbox->{maxlon});
		$bbox->{minlat}=$a->{lat} if($a->{lat}<$bbox->{minlat});
		$bbox->{maxlat}=$a->{lat} if($a->{lat}>$bbox->{maxlat});
	}
	return $bbox;
}

sub utc_time {
	time2str('%Y-%m-%dT%H:%M:%SZ', shift, 'GMT');
}

sub filename_time {
	time2str('%d-%m-%y_%H:%M', shift);
}

sub check_crc
{
	my $data=shift;
	my $cc=hex shift;
	return ($cc == calc_crc($data));
}

sub calc_crc
{
	my $data=shift;
	my $crc=0xffffffff;
	my ($ecx,$edx);
	for(my $i=0;$i<length $data;$i++)
	{
		$ecx=$edx=$crc;
		$ecx>>=8;
		$edx&=0xff;
		$edx^=unpack('C',substr($data,$i,1));
		$ecx^=unpack('V',substr($KEY,$edx*4,4));
		$crc=$ecx;
	}
	return ($crc);
}

sub calc_distance($$)
{
	my $p1=shift;
	my $p2=shift;
	return 0 if(!(defined($p1->{lon})&&defined($p2->{lon})));
	my $lat1 = $p1->{lat};
	my $lat2 = $p2->{lat};
	my $lon1 = $p1->{lon};
	my $lon2 = $p2->{lon};
	my $R = 6378.137; # Radius of earth in KM
	my $dLat = ($lat2 - $lat1) * $PI / 180;
	my $dLon = ($lon2 - $lon1) * $PI / 180;
	my $a = sin($dLat / 2) * sin($dLat / 2) +
		cos($lat1 * $PI / 180) * cos($lat2 * $PI / 180) *
		sin($dLon / 2) * sin($dLon / 2);
	my $c = 2 * atan2(sqrt($a), sqrt(1 - $a));
	my $d = $R * $c;
	return $d*1000; #meters
}


sub calc_length($)
{
	my $track=shift;
	my @point=@{$track->{points}};
	$point[0]->{distance}=0;
	my $distance;
	for(my $i=1;$i<scalar @point;$i++)
	{
		$distance+=calc_distance($point[$i-1],$point[$i]);
		$point[$i]->{distance}=sprintf("%.0d",$distance);
	}
	$track->{length}=$distance;
}
sub calc_ascend($)
{
	my $track=shift;
	my @point=@{$track->{points}};
	if(!defined($point[scalar(@point)-1]->{ele}) && ! defined ($point[0]->{ele}))
	{
		$track->{ascend}=0;
		return 0;
	}
	my $a=$point[scalar(@point)-1]->{ele}-$point[0]->{ele};
	$track->{ascend}=$a;
	return $a;
}

sub calc_time($)
{
	my $track=shift;
	my @point=@{$track->{points}};
	my $starttime=$point[0]->{time}//0;
	for(my $i=0;$i<scalar(@point);$i++)
	{
		my $p=$point[$i];
		if (defined($p->{time})) {
			$p->{time}-=$starttime;
		}
		else #calc time. Suppose speed 10 km/h, 2.7 m/s
		{
			$p->{time}=sprintf("%.0d",$p->{distance}/2.7)||0;
		}
	}
	$track->{time}=$point[scalar(@point)-1]->{time};
	return $track->{time};
}

sub calc_poi($)
{
	my $track=shift;
	my @point=@{$track->{points}};
	$track->{pois}=[];
	foreach(@point)
	{
		if(defined($_->{name}))
		{
			my $p={};
			$p->{name}=substr($_->{name},0,14);
			push @{$track->{pois}},$p;
		}
	}
}

sub open_gpx_file
{
	my $filename=shift;
	@WPT=();
	@TRACK=();
	open my $f, "<$filename" or die $!;
	my $data=join '',<$f>;
	close $f;

	if ($data=~m/(<\?xml.+\?>[^<]*<gpx [^>]+>)/s) {
		my $a=quotemeta($1);
		$data=~s/$a//m;
	}
	if ($data=~m/(<metadata>.+<\/metadata>)/s) {
		my $a=quotemeta($1);
		$data=~s/$a//m;
	}
	if ($data=~m/(<wpt.+<\/wpt>)/s) {
		my @p=split '</wpt>',$1;
		my $a=quotemeta($1);
		$data=~s/$a//m;
		foreach (@p)
		{
			my $p={};
			if (m/<wpt([^>]+)>/s) {
			my $t=$1;
			$p->{lat}=$1 if ($t=~m/lat\W*=\W*"?([\d,\.]+)"?/);
			$p->{lon}=$1 if ($t=~m/lon\W*=\W*"?([\d,\.]+)"?/);
		}
		$p->{ele}=$1 if (m/<ele>\W*([\d,\.]+)\W*<\/ele>/s);
		$p->{ele}=$p->{ele}//0;
		$p->{name}=$1 if (m/<name>([^<]+)<\/name>/s);
		$p->{name}=~s/&#x([0-9a-f]+);/chr(hex($1))/ige;#decode xml encoding
		utf8::decode($p->{name});
		push @WPT,$p;
		}
	}
	if ($data=~m/(<trk>.+<\/trk>)/s) {
		my @track=split '</trk>',$1;
		my $a=quotemeta($1);
		$data=~s/$a//m;
		foreach (@track)
		{
			my $t={};
			$t->{name}=$1 if (m/<name>([^<]+)<\/name>/s);
			$t->{name}=~s/&#x([0-9a-f]+);/chr(hex($1))/ige;#decode xml encoding
			utf8::decode($t->{name});
			if (m/<trkseg>(.+)<\/trkseg>/s) {
				$t->{points}=[];
				my @p=split '</trkpt>',$1;
				foreach (@p) {
					my $p={};
					if (m/<trkpt([^>]+)>/s) {
						my $t=$1;
						$p->{lat}=$1 if ($t=~m/lat\W*=\W*"?([\d,\.]+)"?/);
						$p->{lon}=$1 if ($t=~m/lon\W*=\W*"?([\d,\.]+)"?/);
					}
					$p->{ele}=$1 if (m/<ele>\W*([\d,\.]+)\W*<\/ele>/s);
					$p->{ele}//=0;
					$p->{time}=str2time($1) if (m/<time>\W*([^<]+)\W*<\/time>/s);
					$p->{name}=$1 if (m/<name>([^<]+)<\/name>/s);
					$p->{speed}=$1 if (m/<speed>([^<]+)<\/speed/s);
					$p->{speed}//=10;
					push @{$t->{points}},$p if(defined ($p->{lon} && defined($p->{lat})));
				}
			}
			push @TRACK,$t;
		}
	}
	foreach (@TRACK)
	{
		calc_length($_);
		calc_ascend($_);
		calc_time($_);
		calc_poi($_);
	}
	
}
sub generate_placemarks_bin
{
	my $data;
	foreach my $p (@WPT)
	{
		$data.=chr(0);
		$data.=chr(length $p->{name});
		$data.=$p->{name};
		$data.=chr(0) for(length $p->{name}..237);#fill with zero
		$data.=pack "f",$p->{lat};
		$data.=pack "f",$p->{lon};
		$data.=pack "v",$p->{ele};
		$data.=chr(0).chr(0);
		$data.=pack "V",$PLACEMARK_DELIMETER;
	}
	
	if (scalar(@WPT) <1) { #no placemarks
		$data.=chr(0);
		$data.=chr(5);
		$data.='(0,0)';
		$data.=chr(0) for(5..237);#fill with zero
		$data.=pack "f",0;
		$data.=pack "f",0;
		$data.=pack "v",0;
		$data.=chr(0).chr(0);
		$data.=pack "V",$PLACEMARK_EMPTY;
	}
	
	
	#print $data;
	return $data;
}

sub generate_track_bin
{
	my $data=pack "V", scalar @TRACK; #number of tracks
	#header (32 bytes per track)
	foreach my $t (@TRACK)
	{
		$t->{name_offset}=length $data;
		$data.=pack "V",0x11111111; #track name ptr; byte 0
		$data.=pack "V",$t->{time};
		$data.=pack "V",$t->{length};
		$data.=pack "i",$t->{ascend};
		$data.=pack "V",scalar(@{$t->{points}})+scalar(@{$t->{pois}});
		$t->{start_offset}=length $data;
		$data.=pack "V",0x11111111; #start ptr; byte 20
		$data.=pack "V",scalar(@{$t->{pois}});
		$t->{poi_names_offset}=length $data;
		$data.=pack "V", 0x11111111; #poi names ptr; byte 28
	}
	#points (20 bytes per point)
	foreach my $t (@TRACK)
	{
		substr($data,$t->{start_offset},4)=pack "V", length $data;
		foreach my $p (@{$t->{points}})
		{
			my $point="";
			$point.=pack "f",$p->{lat};
			$point.=pack "f",$p->{lon};
			$point.=pack "v",$p->{ele};
			$point.=pack "v",$p->{speed}*36;
			$point.=pack "V",$p->{time};
			$point.=pack "V",$p->{distance};
			if( defined $p->{name})
			{
				$point.=pack "f",$p->{lat};
				$point.=pack "f",$p->{lon};
				$point.=pack "v",$p->{ele};
				$point.=pack "v",$p->{speed}*36+1;
				$point.=pack "V",$p->{time};
				$point.=pack "V",$p->{distance};
			}
			$data.=$point;
		}
	}
	#footer (20 bytes per entry)
	foreach my $t (@TRACK)
	{
		substr($data,$t->{name_offset},4)=pack "V", length $data;
		$data.=chr(0);
		$data.=chr(length $t->{name});
		$data.=$t->{name};
		$data.=chr(0) for(length $t->{name}..$MAX_TRACK_NAME-1);#fill with zero
		if (scalar(@{$t->{pois}})>0) {
			substr($data,$t->{poi_names_offset},4)=pack "V", length $data;
			foreach my $p (@{$t->{pois}})
			{
				$data.=chr(0);
				$data.=chr(length $p->{name});
				$data.=$p->{name};
				$data.=chr(0) for(length $p->{name}..$MAX_TRACK_NAME-1);#fill with zero
			}
		}
	}
	return $data;
}
